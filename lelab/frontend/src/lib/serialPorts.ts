/**
 * Filter raw `/available-ports` output so USB/serial ports surface cleanly
 * without dozens of Linux virtual consoles (`/dev/tty0..63`) or unused UARTs.
 */
export function filterSelectableSerialPorts(ports: readonly string[]): string[] {
  const unique = Array.from(new Set(ports.filter((port) => !/^\/dev\/tty\d+$/.test(port))));
  const nonLegacyUart = unique.filter((port) => !/^\/dev\/ttyS\d+$/.test(port));
  return nonLegacyUart.length > 0 ? nonLegacyUart : unique;
}
