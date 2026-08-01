# Telegrip Quest teleoperation

This folder is a standalone WebXR teleoperation add-on for the two Dual
Scorpion follower arms. It contains only the Quest/keyboard input path,
Cartesian IK, the arm hardware adapter, the browser UI, and the two URDFs
needed by the IK solver. It does not include recording, replay, training,
policies, cameras, speech, mobile-base, or lift control.

The implementation is adapted from the MIT-licensed
[Telegrip project](https://github.com/DipFlip/telegrip). See [LICENSE](LICENSE).
The URDF assets retain their Apache-2.0 license in [URDF/LICENSE](URDF/LICENSE).

## Safety

- Put both arms in a supported rest pose and clear the workspace before
  connecting.
- Test first with `telegrip --no-robot --no-viz`.
- Keep a person next to the robot and within reach of power.
- The browser is not an emergency stop. Remove motor power for an emergency.
- Robot connection is deliberately manual: open the web UI and select
  **Connect Robot** only when it is safe for the arms to move.

## Install

Telegrip reuses this repository's existing Dual Scorpion follower driver; it
does not ship another robot implementation.

```bash
git clone https://github.com/momoiorg-repository/dual_scorpion.git
cd dual_scorpion

python3.12 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -e ".[core_scripts,feetech]"
pip install -e ./telegrip_teleoperation
```

Open `telegrip_teleoperation/config.yaml` and set:

```yaml
robot:
  left_arm:
    port: /dev/serial/by-id/<left-controller>
  right_arm:
    port: /dev/serial/by-id/<right-controller>
```

Use `lerobot-find-port` to identify each controller. Calibrate the follower
with the repository's normal `lerobot-calibrate` workflow before connecting
Telegrip. No calibration files, serial numbers, certificates, or telemetry
logs are included in this folder.

## Local Quest teleoperation

Local mode has the lowest latency.

1. Connect the Quest and robot computer to the same LAN/Wi-Fi.
2. From the repository root:

   ```bash
   source .venv/bin/activate
   cd telegrip_teleoperation
   ./scripts/start_local.sh
   ```

3. On the Quest, open the printed `https://<robot-ip>:8443` URL. You can also
   use the stable bookmark below after the script updates it.
4. Accept the self-signed certificate warning, select **Enter VR**, verify both
   controllers track, and then select **Connect Robot**.

If the Quest cached an older UI, close the tab and reopen it or perform a hard
refresh.

## Remote Quest teleoperation

Remote mode creates temporary Cloudflare Quick Tunnel URLs for the HTTPS UI
and secure WebSocket. Quick Tunnels are convenient but do not add application
authentication; anyone who receives the active URL can reach the teleoperation
UI. Use a named Cloudflare Tunnel with access controls for unattended or
production use.

Install `cloudflared` and authenticate GitHub CLI once:

```bash
gh auth login
```

Then run:

```bash
source .venv/bin/activate
cd telegrip_teleoperation
./scripts/start_remote.sh
```

The script:

1. starts Telegrip locally;
2. creates separate HTTPS and WebSocket tunnels;
3. writes the current WebSocket hostname to `web-ui/ws_config.js`;
4. updates the stable Quest gist with local and remote links; and
5. stops both tunnels when Telegrip exits or you press Ctrl+C.

## Quest bookmark / GitHub gist

Bookmark this one time in Meta Quest Browser:

<https://gist.github.com/ammarjmahmood/a84200f82fd9618213d05f27e1c255ff>

The gist is secret/unlisted, not access-controlled. Publishing or sharing that
URL allows others to view its current link. The helper uses gist ID
`a84200f82fd9618213d05f27e1c255ff`; override it with
`TELEGRIP_GIST_ID=<id>` if you create a different gist.

For a manual update:

```bash
./scripts/publish_quest_link.sh \
  "https://<current-remote-host>.trycloudflare.com" \
  "https://<robot-lan-ip>:8443"
```

## Quest controls

- Hold left/right **grip**: move that arm relative to the controller pose.
- Hold left/right **trigger**: close that gripper; release to open.
- Hold right **A + B** for 3 seconds: home both arms.
- Hold left **X + Y** for 3 seconds: move both arms to the rest pose.
- Click either thumbstick: toggle the robot diagnostic shadow.

Both controllers use full-pose IK with all seven body joints enabled. No body
joint is locked in the default configuration.

Use `control.operator_to_robot_yaw_deg` in `config.yaml` to align operator
forward with the robot frame. Change it in 90-degree steps and restart.

## Useful commands

```bash
# Browser/IK test without hardware
telegrip --no-robot --no-viz

# Direct startup without the helper scripts
telegrip --no-viz \
  --left-port /dev/serial/by-id/<left-controller> \
  --right-port /dev/serial/by-id/<right-controller>
```

Runtime TLS files (`cert.pem`, `key.pem`), generated tunnel configuration, and
session telemetry are ignored by Git.
