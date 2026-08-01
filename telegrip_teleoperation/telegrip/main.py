"""
Main entry point for the unified teleoperation system.
Coordinates HTTPS server, WebSocket server, robot interface, and input providers.
"""

import asyncio
import argparse
import logging
import signal
import sys
import os
import http.server
import ssl
import socket
import json
import urllib.parse
import time
import contextlib
from typing import Optional
import queue  # Add regular queue for thread-safe communication
import threading
from pathlib import Path
import weakref


def get_local_ip():
    """Get the local IP address of this machine."""
    try:
        # Connect to a remote address to determine the local IP
        # This doesn't actually send any data
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("8.8.8.8", 80))
            return s.getsockname()[0]
    except Exception:
        try:
            # Fallback: get hostname IP
            return socket.gethostbyname(socket.gethostname())
        except Exception:
            # Final fallback
            return "localhost"


@contextlib.contextmanager
def suppress_stdout_stderr():
    """Context manager to suppress stdout and stderr output at the file descriptor level."""
    # Save original file descriptors
    stdout_fd = sys.stdout.fileno()
    stderr_fd = sys.stderr.fileno()
    
    # Save original file descriptors
    saved_stdout_fd = os.dup(stdout_fd)
    saved_stderr_fd = os.dup(stderr_fd)
    
    try:
        # Open devnull
        devnull_fd = os.open(os.devnull, os.O_WRONLY)
        
        # Redirect stdout and stderr to devnull
        os.dup2(devnull_fd, stdout_fd)
        os.dup2(devnull_fd, stderr_fd)
        
        yield
        
    finally:
        # Restore original file descriptors
        os.dup2(saved_stdout_fd, stdout_fd)
        os.dup2(saved_stderr_fd, stderr_fd)
        
        # Close saved file descriptors
        os.close(saved_stdout_fd)
        os.close(saved_stderr_fd)
        os.close(devnull_fd)


# Import telegrip modules after function definition
from .config import TelegripConfig, get_config_data, update_config_data, HOME_POSE_PRESETS
from .control_loop import ControlLoop
from .inputs.vr_ws_server import VRWebSocketServer
from .inputs.web_keyboard import WebKeyboardHandler
from .inputs.base import ControlGoal
from .telemetry import TelemetryRecorder

# Logger will be configured in main() based on command line arguments
logger = logging.getLogger(__name__)


class APIHandler(http.server.BaseHTTPRequestHandler):
    """HTTP request handler for the teleoperation API."""
    
    def __init__(self, *args, **kwargs):
        # Set CORS headers for all requests
        super().__init__(*args, **kwargs)
    
    def end_headers(self):
        """Add CORS headers to all responses."""
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        try:
            super().end_headers()
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, ssl.SSLError):
            # Client disconnected or SSL error - ignore silently
            pass
    
    def do_OPTIONS(self):
        """Handle preflight CORS requests."""
        self.send_response(200)
        self.end_headers()
    
    def log_message(self, format, *args):
        """Override to reduce HTTP request logging noise."""
        pass  # Disable default HTTP logging
    
    def do_GET(self):
        """Handle GET requests."""
        if self.path == '/api/status':
            self.handle_status_request()
        elif self.path == '/api/config':
            self.handle_config_get_request()
        elif self.path == '/' or self.path == '/index.html':
            # Serve main page from web-ui directory
            self.serve_file('web-ui/index.html', 'text/html')
        elif self.path.endswith('.css'):
            # Serve CSS files from web-ui directory
            self.serve_file(f'web-ui{self.path}', 'text/css')
        elif self.path.endswith('.js'):
            # Serve JS files from web-ui directory
            self.serve_file(f'web-ui{self.path}', 'application/javascript')
        elif self.path.endswith('.ico'):
            self.serve_file(self.path[1:], 'image/x-icon')
        elif self.path.endswith(('.jpg', '.jpeg')):
            # Serve image files from web-ui directory
            self.serve_file(f'web-ui{self.path}', 'image/jpeg')
        elif self.path.endswith('.png'):
            # Serve image files from web-ui directory
            self.serve_file(f'web-ui{self.path}', 'image/png')
        elif self.path.endswith('.gif'):
            # Serve image files from web-ui directory
            self.serve_file(f'web-ui{self.path}', 'image/gif')
        else:
            self.send_error(404, "Not found")
    
    def do_POST(self):
        """Handle POST requests."""
        if self.path == '/api/keyboard':
            self.handle_keyboard_request()
        elif self.path == '/api/robot':
            self.handle_robot_request()
        elif self.path == '/api/keypress':
            self.handle_keypress_request()
        elif self.path == '/api/config':
            self.handle_config_post_request()
        elif self.path == '/api/restart':
            self.handle_restart_request()
        else:
            self.send_error(404, "Not found")
    
    def handle_status_request(self):
        """Handle status requests."""
        try:
            # Get system reference
            if hasattr(self.server, 'api_handler') and self.server.api_handler:
                system = self.server.api_handler
                
                # Get status from control loop
                control_status = system.control_loop.status if system.control_loop else {}
                
                # Get keyboard status
                keyboard_enabled = False
                if system.web_keyboard_handler and hasattr(system.web_keyboard_handler, 'is_enabled'):
                    keyboard_enabled = system.web_keyboard_handler.is_enabled
                
                # Get robot engagement status
                robot_engaged = False
                if system.control_loop and system.control_loop.robot_interface:
                    robot_engaged = system.control_loop.robot_interface.is_engaged
                
                # Get VR connection status
                vr_connected = False
                if system.vr_server and system.vr_server.is_running:
                    vr_connected = len(system.vr_server.clients) > 0
                
                status = {
                    **control_status,
                    "keyboardEnabled": keyboard_enabled,
                    "robotEngaged": robot_engaged,
                    "vrConnected": vr_connected
                }
                
                # Send JSON response
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.end_headers()
                
                response = json.dumps(status)
                self.wfile.write(response.encode('utf-8'))
            else:
                self.send_error(500, "System not available")
                
        except Exception as e:
            logger.error(f"Error handling status request: {e}")
            self.send_error(500, str(e))
    
    def handle_keyboard_request(self):
        """Handle keyboard control requests."""
        try:
            content_length = int(self.headers.get('Content-Length', 0))
            if content_length == 0:
                self.send_error(400, "No request body")
                return
            
            post_data = self.rfile.read(content_length)
            data = json.loads(post_data.decode('utf-8'))
            
            action = data.get('action')
            
            if action in ['enable', 'disable']:
                # Add keyboard command to queue for processing by main thread
                if hasattr(self.server, 'api_handler') and self.server.api_handler:
                    command_name = f"{action}_keyboard"
                    logger.info(f"🎮 Adding command to queue: {command_name}")
                    self.server.api_handler.add_control_command(command_name)
                    
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json')
                    self.end_headers()
                    self.wfile.write(json.dumps({"success": True, "action": action}).encode('utf-8'))
                else:
                    self.send_error(500, "System not available")
            else:
                self.send_error(400, f"Invalid action: {action}")
                
        except json.JSONDecodeError:
            self.send_error(400, "Invalid JSON")
        except Exception as e:
            logger.error(f"Error handling keyboard request: {e}")
            self.send_error(500, str(e))
    
    def handle_robot_request(self):
        """Handle robot control requests."""
        try:
            content_length = int(self.headers.get('Content-Length', 0))
            if content_length == 0:
                self.send_error(400, "No request body")
                return
            
            post_data = self.rfile.read(content_length)
            data = json.loads(post_data.decode('utf-8'))
            
            action = data.get('action')
            logger.info(f"🔌 Received robot action: {action}")
            
            if action in ['connect', 'disconnect']:
                # Add robot command to queue for processing by main thread
                if hasattr(self.server, 'api_handler') and self.server.api_handler:
                    command_name = f"robot_{action}"
                    logger.info(f"🔌 Adding command to queue: {command_name}")
                    self.server.api_handler.add_control_command(command_name)
                    
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json')
                    self.end_headers()
                    self.wfile.write(json.dumps({"success": True, "action": action}).encode('utf-8'))
                else:
                    logger.error("🔌 Server api_handler not available")
                    self.send_error(500, "System not available")
            else:
                self.send_error(400, f"Invalid action: {action}")
                
        except json.JSONDecodeError:
            self.send_error(400, "Invalid JSON")
        except Exception as e:
            logger.error(f"Error handling robot request: {e}")
            self.send_error(500, str(e))
    
    def handle_keypress_request(self):
        """Handle keypress control requests."""
        try:
            content_length = int(self.headers.get('Content-Length', 0))
            if content_length == 0:
                self.send_error(400, "No request body")
                return
            
            post_data = self.rfile.read(content_length)
            data = json.loads(post_data.decode('utf-8'))
            
            key = data.get('key')
            action = data.get('action')
            
            if key and action in ['press', 'release']:
                # Add keypress command to queue for processing by main thread
                if hasattr(self.server, 'api_handler') and self.server.api_handler:
                    command = {
                        "action": "web_keypress",
                        "key": key,
                        "event": action
                    }
                    logger.info(f"🎮 Adding keypress command to queue: {key}_{action}")
                    self.server.api_handler.add_keypress_command(command)
                    
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json')  
                    self.end_headers()
                    self.wfile.write(json.dumps({"success": True, "key": key, "action": action}).encode('utf-8'))
                else:
                    logger.error("🎮 Server api_handler not available")
                    self.send_error(500, "System not available")
            else:
                self.send_error(400, f"Invalid key or action: {key}, {action}")
                
        except json.JSONDecodeError:
            self.send_error(400, "Invalid JSON")
        except Exception as e:
            logger.error(f"Error handling keypress request: {e}")
            self.send_error(500, str(e))
    
    def handle_config_get_request(self):
        """Handle configuration read requests."""
        try:
            config_data = get_config_data()
            
            # Send JSON response
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            
            response = json.dumps(config_data)
            self.wfile.write(response.encode('utf-8'))
            
        except Exception as e:
            logger.error(f"Error handling config get request: {e}")
            self.send_error(500, str(e))
    
    def handle_config_post_request(self):
        """Handle configuration update requests."""
        try:
            content_length = int(self.headers.get('Content-Length', 0))
            if content_length == 0:
                self.send_error(400, "No request body")
                return
            
            post_data = self.rfile.read(content_length)
            data = json.loads(post_data.decode('utf-8'))
            
            # Update configuration
            success = update_config_data(data)
            
            if success:
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps({"success": True, "message": "Configuration updated successfully"}).encode('utf-8'))
                logger.info("Configuration updated successfully")
            else:
                self.send_error(500, "Failed to save configuration")
                
        except json.JSONDecodeError:
            self.send_error(400, "Invalid JSON")
        except Exception as e:
            logger.error(f"Error handling config post request: {e}")
            self.send_error(500, str(e))
    
    def handle_restart_request(self):
        """Handle restart requests."""
        try:
            if hasattr(self.server, 'api_handler') and self.server.api_handler:
                logger.info("Restarting teleoperation system...")
                self.server.api_handler.restart()
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps({"success": True, "message": "Teleoperation system restarted"}).encode('utf-8'))
            else:
                self.send_error(500, "System not available")
                
        except Exception as e:
            logger.error(f"Error handling restart request: {e}")
            self.send_error(500, str(e))
    
    def serve_file(self, filename, content_type):
        """Serve a static file from the project directory."""
        from .utils import get_absolute_path
        try:
            # Convert relative path to absolute path in project directory
            abs_path = get_absolute_path(filename)
            
            with open(abs_path, 'rb') as f:
                file_content = f.read()
            
            self.send_response(200)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', len(file_content))
            self.end_headers()
            self.wfile.write(file_content)
            
        except FileNotFoundError:
            self.send_error(404, f"File {filename} not found")
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            # Client disconnected - log quietly and continue
            logger.debug(f"Client disconnected while serving {filename}")
        except Exception as e:
            logger.error(f"Error serving file {filename}: {e}")
            try:
                self.send_error(500, "Internal server error")
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                # Client already disconnected, ignore
                pass


class ThreadingHTTPSServer(http.server.ThreadingHTTPServer):
    """ThreadingHTTPServer that performs the TLS handshake per-connection (not on the
    listening socket), with a bounded handshake timeout. A socket's settimeout() is not
    inherited by the sockets accept() returns, so the timeout must be applied directly to
    each newly-accepted socket - otherwise a client that never completes the TLS handshake
    blocks get_request() (and therefore every other client) indefinitely."""
    request_queue_size = 64
    handshake_timeout = 10

    def __init__(self, server_address, handler_cls, ssl_context):
        self.ssl_context = ssl_context
        super().__init__(server_address, handler_cls)

    def get_request(self):
        newsocket, fromaddr = self.socket.accept()
        newsocket.settimeout(self.handshake_timeout)
        ssl_socket = self.ssl_context.wrap_socket(newsocket, server_side=True)
        ssl_socket.settimeout(None)
        return ssl_socket, fromaddr


class HTTPSServer:
    """HTTPS server for the teleoperation API."""

    def __init__(self, config: TelegripConfig):
        self.config = config
        self.httpd = None
        self.server_thread = None
        self.system_ref = None  # Direct reference to the main system

    def set_system_ref(self, system_ref):
        """Set reference to the main teleoperation system."""
        self.system_ref = system_ref

    async def start(self):
        """Start the HTTPS server."""
        try:
            # Setup SSL
            context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            # Get absolute paths for SSL certificates
            cert_path, key_path = self.config.get_absolute_ssl_paths()
            context.load_cert_chain(cert_path, key_path)

            # Use a threading server so one client doesn't block others while being handled
            self.httpd = ThreadingHTTPSServer((self.config.host_ip, self.config.https_port), APIHandler, context)

            # Set API handler reference for command queuing
            self.httpd.api_handler = self.system_ref

            # Start server in a separate thread
            self.server_thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
            self.server_thread.start()
            
            # Only log if INFO level or more verbose
            if getattr(logging, self.config.log_level.upper()) <= logging.INFO:
                host_display = get_local_ip() if self.config.host_ip == "0.0.0.0" else self.config.host_ip
                logger.info(f"HTTPS server started on https://{host_display}:{self.config.https_port}")
            
        except Exception as e:
            logger.error(f"Failed to start HTTPS server: {e}")
            raise
    
    async def stop(self):
        """Stop the HTTPS server."""
        if self.httpd:
            self.httpd.shutdown()
            if self.server_thread:
                self.server_thread.join(timeout=5)
            logger.info("HTTPS server stopped")


class TelegripSystem:
    """Main teleoperation system that coordinates all components."""
    
    def __init__(self, config: TelegripConfig):
        self.config = config
        
        # Command queues
        self.command_queue = asyncio.Queue()
        self.control_commands_queue = queue.Queue(maxsize=10)  # Thread-safe queue

        self.telemetry = TelemetryRecorder(
            enabled=config.telemetry_enabled,
            log_dir=config.telemetry_log_dir,
        )
        
        # Components
        self.https_server = HTTPSServer(config)
        self.vr_server = VRWebSocketServer(self.command_queue, config, self.telemetry)
        self.web_keyboard_handler = WebKeyboardHandler(self.command_queue, config)
        self.control_loop = ControlLoop(
            self.command_queue,
            config,
            self.control_commands_queue,
            self.telemetry,
        )

        # Set system reference for API calls
        self.https_server.set_system_ref(self)

        # Set up cross-references
        self.control_loop.web_keyboard_handler = self.web_keyboard_handler
        self.control_loop.vr_server = self.vr_server  # VR skeleton broadcast

        # Set up disconnect callback for ESC key
        self.web_keyboard_handler.disconnect_callback = lambda: self.add_control_command("robot_disconnect")
        
        # Tasks
        self.tasks = []
        self.is_running = False
        self.main_loop = None  # Will be set when the system starts
    
    def add_control_command(self, action: str):
        """Add a control command to the queue for processing."""
        try:
            command = {"action": action}
            logger.info(f"🔌 Queueing control command: {command}")
            self.control_commands_queue.put_nowait(command)
            logger.info(f"🔌 Command queued successfully")
        except queue.Full:
            logger.warning(f"Control commands queue is full, dropping command: {action}")
        except Exception as e:
            logger.error(f"🔌 Error queuing command: {e}")
    
    def add_keypress_command(self, command: dict):
        """Add a keypress command to the queue for processing."""
        try:
            logger.info(f"🎮 Queueing keypress command: {command}")
            self.control_commands_queue.put_nowait(command)
            logger.info(f"🎮 Keypress command queued successfully")
        except queue.Full:
            logger.warning(f"Control commands queue is full, dropping keypress command: {command}")
        except Exception as e:
            logger.error(f"🎮 Error queuing keypress command: {e}")
    
    async def process_control_commands(self):
        """Process control commands from the thread-safe queue."""
        try:
            # Get all available commands from the thread-safe queue
            commands_to_process = []
            while True:
                try:
                    command = self.control_commands_queue.get_nowait()
                    commands_to_process.append(command)
                except queue.Empty:
                    break
            
            # Process each command
            for command in commands_to_process:
                if self.control_loop:
                    await self.control_loop._handle_command(command)
                    
        except Exception as e:
            logger.error(f"Error processing control commands: {e}")
    
    def restart(self):
        """Restart the teleoperation system."""
        def do_restart():
            try:
                logger.info("Initiating system restart...")
                # Use the stored main event loop reference to schedule the soft restart
                if self.main_loop and not self.main_loop.is_closed():
                    future = asyncio.run_coroutine_threadsafe(self._soft_restart_sequence(), self.main_loop)
                    # Wait for restart to complete
                    future.result(timeout=30.0)
                else:
                    logger.error("Main event loop not available for restart")
            except Exception as e:
                logger.error(f"Error during restart: {e}")
        
        # Run restart in a separate thread to avoid blocking the HTTP response
        restart_thread = threading.Thread(target=do_restart, daemon=True)
        restart_thread.start()
    
    async def _soft_restart_sequence(self):
        """Perform a soft restart by reinitializing components without exiting the process."""
        try:
            logger.info("Starting soft restart sequence...")
            
            # Wait a moment to let the HTTP response be sent
            await asyncio.sleep(1)
            
            # Cancel all tasks
            for task in self.tasks:
                task.cancel()
            
            # Wait for tasks to complete with timeout
            if self.tasks:
                try:
                    await asyncio.wait_for(
                        asyncio.gather(*self.tasks, return_exceptions=True), 
                        timeout=5.0
                    )
                except asyncio.TimeoutError:
                    logger.warning("Some tasks did not complete within timeout")
            
            # Stop components in reverse order
            await self.control_loop.stop()
            await self.web_keyboard_handler.stop()
            await self.vr_server.stop()
            # Don't stop HTTPS server - keep it running for the UI

            # Wait a moment for cleanup
            await asyncio.sleep(1)

            # Reload configuration from file but preserve command-line overrides
            from .config import get_config_data
            file_config = get_config_data()
            logger.info("Configuration reloaded from file")

            # Keep the existing configuration object to preserve command-line arguments
            # Just update specific values that might have changed in the config file

            # Recreate components with existing configuration
            self.command_queue = asyncio.Queue()
            self.control_commands_queue = queue.Queue(maxsize=10)

            # Create new components
            self.vr_server = VRWebSocketServer(self.command_queue, self.config, self.telemetry)
            self.web_keyboard_handler = WebKeyboardHandler(self.command_queue, self.config)
            self.control_loop = ControlLoop(
                self.command_queue,
                self.config,
                self.control_commands_queue,
                self.telemetry,
            )

            # Set up cross-references
            self.control_loop.web_keyboard_handler = self.web_keyboard_handler
            self.control_loop.vr_server = self.vr_server  # VR skeleton broadcast

            # Set up disconnect callback for ESC key
            self.web_keyboard_handler.disconnect_callback = lambda: self.add_control_command("robot_disconnect")

            # Clear old tasks
            self.tasks = []

            # Start VR WebSocket server
            await self.vr_server.start()

            # Start web keyboard handler
            await self.web_keyboard_handler.start()

            # Start control loop
            control_task = asyncio.create_task(self.control_loop.start())
            self.tasks.append(control_task)

            # Start control command processor
            command_processor_task = asyncio.create_task(self._run_command_processor())
            self.tasks.append(command_processor_task)

            logger.info("System restart completed successfully")
            
            # Auto-connect to robot if requested (preserve autoconnect behavior after restart)
            if self.config.autoconnect and self.config.enable_robot:
                logger.info("🔌 Auto-connecting to robot motors after restart...")
                await asyncio.sleep(0.5)  # Brief delay to let components settle
                self.add_control_command("robot_connect")
            
        except Exception as e:
            logger.error(f"Error during soft restart sequence: {e}")
            raise
    
    async def _await_ready_gate(self) -> bool:
        """Block until the configured ready sentinel file exists.

        Returns True if the file is present (or no gating is configured), False
        if we time out or shut down first. When False, the caller must NOT home
        the arms - the teleop link is not confirmed live.
        """
        ready_file = getattr(self.config, "wait_ready_file", None)
        if not ready_file:
            return True

        if os.path.exists(ready_file):
            return True

        timeout = float(getattr(self.config, "wait_ready_timeout", 180.0))
        logger.info(f"⏳ Waiting for teleop link to go live before homing arms "
                    f"(file: {ready_file}, timeout {timeout:.0f}s)...")
        waited = 0.0
        interval = 0.5
        while waited < timeout:
            if os.path.exists(ready_file):
                logger.info("✅ Teleop link is live — proceeding to engage and home the arms.")
                return True
            if not self.is_running:
                return False
            await asyncio.sleep(interval)
            waited += interval

        logger.error(f"❌ Teleop link not live after {timeout:.0f}s — NOT homing the arms.")
        return False

    async def start(self):
        """Start all system components."""
        try:
            self.is_running = True
            self.telemetry.start()
            
            # Store reference to the main event loop for restart functionality
            self.main_loop = asyncio.get_event_loop()
            
            # Start HTTPS server
            await self.https_server.start()
            
            # Start VR WebSocket server
            await self.vr_server.start()

            # Start web keyboard handler
            await self.web_keyboard_handler.start()

            # Start control loop
            control_task = asyncio.create_task(self.control_loop.start())
            self.tasks.append(control_task)

            # Start control command processor
            command_processor_task = asyncio.create_task(self._run_command_processor())
            self.tasks.append(command_processor_task)

            logger.info("All system components started successfully")
            
            # Auto-connect to robot if requested
            if self.config.autoconnect and self.config.enable_robot:
                # Gate homing on the teleop link being live (if configured). The
                # arms "going up" is the human-visible signal that teleop is ready,
                # so we must not home until the Cloudflare link is published.
                if await self._await_ready_gate():
                    logger.info("🔌 Auto-connecting to robot motors...")
                    await asyncio.sleep(0.5)  # Brief delay to let components settle
                    self.add_control_command("robot_connect")
                else:
                    logger.warning("🔌 Auto-connect skipped: teleop link not confirmed live. "
                                   "Arms stay parked. Engage manually or relaunch once the link is up.")
            
            # Main loop that handles restarts
            while self.is_running:
                try:
                    # Wait for tasks to complete
                    await asyncio.gather(*self.tasks)
                    # If we get here, all tasks completed normally (shouldn't happen in normal operation)
                    break
                except asyncio.CancelledError:
                    # Tasks were cancelled - check if it's due to restart
                    if self.is_running:
                        # System is restarting, wait for restart to complete
                        await asyncio.sleep(1)
                        # Continue the loop to wait for new tasks
                        continue
                    else:
                        # Normal shutdown
                        break
                except Exception as e:
                    logger.error(f"Error in main task loop: {e}")
                    break
            
        except OSError as e:
            if e.errno == 98:  # Address already in use
                logger.error(f"Error starting teleoperation system: {e}")
                logger.error(f"To find and kill the process using these ports, run:")
                logger.error(f"  kill -9 $(lsof -t -i:{self.config.https_port} -i:{self.config.websocket_port})")
            else:
                logger.error(f"Error starting teleoperation system: {e}")
            await self.stop()
            raise
        except Exception as e:
            logger.error(f"Error starting teleoperation system: {e}")
            await self.stop()
            raise
    
    async def _run_command_processor(self):
        """Run the control command processor loop."""
        while self.is_running:
            await self.process_control_commands()
            await asyncio.sleep(0.05)  # Check for commands every 50ms
    
    async def stop(self):
        """Stop all system components."""
        logger.info("Shutting down teleoperation system...")
        self.is_running = False

        # Stop VR server first to close websocket connections (unblocks any waiting handlers)
        try:
            await asyncio.wait_for(self.vr_server.stop(), timeout=2.0)
        except asyncio.TimeoutError:
            logger.warning("VR server stop timed out")
        except Exception as e:
            logger.warning(f"Error stopping VR server: {e}")

        # Cancel all tasks
        for task in self.tasks:
            task.cancel()

        # Wait for tasks to complete with timeout
        if self.tasks:
            try:
                await asyncio.wait_for(
                    asyncio.gather(*self.tasks, return_exceptions=True),
                    timeout=2.0
                )
            except asyncio.TimeoutError:
                logger.warning("Some tasks did not complete within timeout")

        # Stop remaining components. Allow enough time for the graceful arm park
        # (smooth move to the safe hang-down pose + settle) before torque release;
        # a too-short timeout here would abort the park and let the arms drop.
        try:
            await asyncio.wait_for(self.control_loop.stop(), timeout=10.0)
        except asyncio.TimeoutError:
            logger.warning("Control loop stop timed out")
        except Exception as e:
            logger.warning(f"Error stopping control loop: {e}")

        try:
            await asyncio.wait_for(self.web_keyboard_handler.stop(), timeout=1.0)
        except asyncio.TimeoutError:
            logger.warning("Web keyboard handler stop timed out")
        except Exception as e:
            logger.warning(f"Error stopping web keyboard handler: {e}")

        try:
            await asyncio.wait_for(self.https_server.stop(), timeout=2.0)
        except asyncio.TimeoutError:
            logger.warning("HTTPS server stop timed out")
        except Exception as e:
            logger.warning(f"Error stopping HTTPS server: {e}")

        self.telemetry.stop()
        logger.info("Teleoperation system shutdown complete")


def create_signal_handler(system: 'TelegripSystem', loop: asyncio.AbstractEventLoop):
    """Create a signal handler that properly stops the system."""
    def signal_handler(signum, frame):
        """Request shutdown without interrupting an active motor transaction.

        Raising SystemExit here can land in the middle of a synchronous Feetech
        SDK read/write and leave its serial PortHandler marked ``is_using``.
        Subsequent park and torque-disable writes then all fail with
        ``Port is in use``, leaving an arm locked. Cancelling asyncio tasks is
        safe because cancellation is delivered at their next await boundary,
        after any current synchronous motor transaction has returned.
        """
        logger.info(f"Received signal {signum}")
        if not system.is_running:
            return
        system.is_running = False
        for task in system.tasks:
            loop.call_soon_threadsafe(task.cancel)
    return signal_handler


def parse_arguments():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(description="Unified dual_scorpion Teleoperation System")
    
    # Control flags
    parser.add_argument("--no-robot", action="store_true", help="Disable robot connection (visualization only)")
    parser.add_argument(
        "--no-sim",
        action="store_true",
        help="Deprecated and ignored: the headless PyBullet IK engine is required",
    )
    parser.add_argument("--no-viz", action="store_true", help="Disable PyBullet visualization (headless mode)")
    parser.add_argument("--no-vr", action="store_true", help="Disable VR WebSocket server")
    parser.add_argument("--no-keyboard", action="store_true", help="Disable keyboard input")
    parser.add_argument("--no-https", action="store_true", help="Disable HTTPS server")
    parser.add_argument("--autoconnect", action="store_true", help="Automatically connect to robot motors on startup")
    parser.add_argument(
        "--operator-mode",
        choices=("embodied", "mirror"),
        default=None,
        help="embodied = stand beside/behind the robot; mirror = face the robot",
    )
    parser.add_argument("--start-pose", default=None, choices=sorted(HOME_POSE_PRESETS.keys()),
                       help="Start pose the arms home to on engage and on the A+B VR combo "
                            "(overrides config; default from config.yaml robot.start_pose)")
    parser.add_argument("--wait-ready-file", default=None,
                       help="If set, wait for this file to exist before auto-connecting/homing the arms "
                            "(gates arm homing on the Cloudflare teleop link being live).")
    parser.add_argument("--log-level", default="warning", 
                       choices=["debug", "info", "warning", "error", "critical"],
                       help="Set logging level (default: warning)")
    
    # Network settings
    parser.add_argument("--https-port", type=int, default=8443, help="HTTPS server port")
    parser.add_argument("--ws-port", type=int, default=8442, help="WebSocket server port")
    parser.add_argument("--host", default="0.0.0.0", help="Host IP address")
    
    # Paths
    parser.add_argument("--urdf-left", default=None, help="Path to left arm URDF file (overrides config)")
    parser.add_argument("--urdf-right", default=None, help="Path to right arm URDF file (overrides config)")
    parser.add_argument("--webapp", default="webapp", help="Path to webapp directory")
    parser.add_argument("--cert", default="cert.pem", help="Path to SSL certificate")
    parser.add_argument("--key", default="key.pem", help="Path to SSL private key")
    
    # Robot settings
    parser.add_argument("--config", default="config.yaml", help="Path to config file")
    parser.add_argument("--left-port", help="Left arm serial port (overrides config file)")
    parser.add_argument("--right-port", help="Right arm serial port (overrides config file)")
    
    return parser.parse_args()


def create_config_from_args(args) -> TelegripConfig:
    """Create configuration object from command line arguments."""
    # First load the config file
    config_data = get_config_data()
    config = TelegripConfig()
    
    # Apply command line overrides
    config.enable_robot = not args.no_robot
    # PyBullet supplies the IK solver and is mandatory for Cartesian arm
    # movement. --no-viz disables only the GUI/rendering, leaving low-overhead
    # DIRECT-mode kinematics active. Keep --no-sim as an ignored compatibility
    # flag so older launch scripts cannot accidentally disable arm movement.
    config.enable_pybullet = True
    config.enable_pybullet_gui = not args.no_viz
    if args.no_sim:
        logger.warning("--no-sim is ignored: headless PyBullet IK is required for arm movement")
    config.enable_vr = not args.no_vr
    config.enable_keyboard = not args.no_keyboard
    config.autoconnect = args.autoconnect
    config.wait_ready_file = args.wait_ready_file
    config.log_level = args.log_level
    if args.start_pose:
        config.set_start_pose(args.start_pose)
    if args.operator_mode:
        mirrored = args.operator_mode == "mirror"
        config.mirror_lateral = {"left": mirrored, "right": mirrored}
        logger.info(
            "Operator mode: %s (lateral mirror=%s)",
            args.operator_mode,
            mirrored,
        )
    
    config.https_port = args.https_port
    config.websocket_port = args.ws_port
    config.host_ip = args.host
    
    if args.urdf_left:
        config.urdf_left_path = args.urdf_left
    if args.urdf_right:
        config.urdf_right_path = args.urdf_right
    config.webapp_dir = args.webapp
    config.certfile = args.cert
    config.keyfile = args.key
    
    # Handle port configuration - use command line args if provided, otherwise use config file values
    if args.left_port or args.right_port:
        config.follower_ports = {
            "left": args.left_port if args.left_port else config_data["robot"]["left_arm"]["port"],
            "right": args.right_port if args.right_port else config_data["robot"]["right_arm"]["port"]
        }
    
    return config


async def main():
    """Main entry point."""
    # Parse arguments first to check for log level
    args = parse_arguments()
    
    # Setup logging based on log level
    log_level = getattr(logging, args.log_level.upper())
    
    # Suppress PyBullet's native output when not in verbose mode
    if log_level > logging.INFO:
        os.environ['PYBULLET_SUPPRESS_CONSOLE_OUTPUT'] = '1'
        os.environ['PYBULLET_SUPPRESS_WARNINGS'] = '1'
    
    if log_level <= logging.INFO:
        # Verbose mode - show detailed logging with timestamps
        logging.basicConfig(
            level=log_level,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )
    else:
        # Quiet mode - only show warnings and errors with simple format
        logging.basicConfig(
            level=log_level,
            format='%(message)s'
        )

    # Suppress noisy websockets library logging (invalid HTTP requests to WS port)
    logging.getLogger('websockets').setLevel(logging.WARNING)

    config = create_config_from_args(args)

    # Ensure SSL certificates exist (generate if needed for first-time startup)
    if not config.ensure_ssl_certificates():
        logger.error("Failed to ensure SSL certificates are available")
        sys.exit(1)

    # Log configuration (only if INFO level or more verbose)
    if log_level <= logging.INFO:
        logger.info("Starting with configuration:")
        logger.info(f"  Robot: {'enabled' if config.enable_robot else 'disabled'}")
        logger.info(f"  PyBullet: {'enabled' if config.enable_pybullet else 'disabled'}")
        logger.info(f"  Headless mode: {'enabled' if not config.enable_pybullet_gui and config.enable_pybullet else 'disabled'}")
        logger.info(f"  VR: {'enabled' if config.enable_vr else 'disabled'}")
        logger.info(f"  Keyboard: {'enabled' if config.enable_keyboard else 'disabled'}")
        logger.info(f"  Auto-connect: {'enabled' if config.autoconnect else 'disabled'}")
        logger.info(f"  HTTPS Port: {config.https_port}")
        logger.info(f"  WebSocket Port: {config.websocket_port}")
        logger.info(f"  Robot Ports: {config.follower_ports}")
    else:
        # Show clean startup message with HTTPS URL
        host_display = get_local_ip() if config.host_ip == "0.0.0.0" else config.host_ip
        print(f"🤖 telegrip starting...")
        print(f"📱 Open the UI in your browser on:")
        print(f"   https://{host_display}:{config.https_port}")
        print(f"📱 Then go to the same address on your VR headset browser")
        print(f"💡 Use --log-level info to see detailed output")
        print()
    
    # Create and start teleoperation system
    system = TelegripSystem(config)

    # Setup signal handlers with reference to system and event loop
    loop = asyncio.get_event_loop()
    signal_handler = create_signal_handler(system, loop)
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        await system.start()
    except (KeyboardInterrupt, SystemExit):
        if log_level <= logging.INFO:
            logger.info("Received interrupt signal")
        else:
            print("\n🛑 Shutting down...")
    except asyncio.CancelledError:
        # Handle cancelled error (often from restart scenarios)
        if log_level <= logging.INFO:
            logger.info("System tasks cancelled")
    except Exception as e:
        if log_level <= logging.INFO:
            logger.error(f"System error: {e}")
        else:
            print(f"❌ Error: {e}")
    finally:
        try:
            await system.stop()
        except (asyncio.CancelledError, SystemExit):
            # Ignore cancelled/exit errors during shutdown
            pass

        # Suppress SSL transport errors during event loop cleanup
        def ignore_ssl_errors(loop, context):
            # Ignore "Bad file descriptor" and "Event loop is closed" errors during shutdown
            if 'exception' in context:
                exc = context['exception']
                if isinstance(exc, (OSError, RuntimeError)):
                    return
            # Log other errors normally
            loop.default_exception_handler(context)

        loop.set_exception_handler(ignore_ssl_errors)

        if log_level > logging.INFO:
            print("✅ Shutdown complete.")


def main_cli():
    """Console script entry point for pip-installed package."""
    try:
        asyncio.run(main())
    except (KeyboardInterrupt, SystemExit):
        print("\nShutdown complete.")
    except asyncio.CancelledError:
        # Handle cancelled error from restart scenarios
        pass
    except Exception as e:
        logger.error(f"Fatal error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main_cli() 
