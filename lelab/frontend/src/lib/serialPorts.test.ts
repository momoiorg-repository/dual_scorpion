import { describe, expect, test } from "vitest";
import { filterSelectableSerialPorts } from "./serialPorts";

describe("filterSelectableSerialPorts", () => {
  test("drops Linux virtual terminals and hides ttyS when USB serial ports exist", () => {
    const raw = [
      "/dev/tty0",
      "/dev/tty1",
      "/dev/ttyS0",
      "/dev/ttyS31",
      "/dev/ttyACM0",
      "/dev/ttyACM1",
      "/dev/ttyUSB0",
      "/dev/ttyACM0",
    ];
    expect(filterSelectableSerialPorts(raw)).toEqual([
      "/dev/ttyACM0",
      "/dev/ttyACM1",
      "/dev/ttyUSB0",
    ]);
  });

  test("falls back to ttyS ports when no USB serial ports exist", () => {
    expect(filterSelectableSerialPorts(["/dev/tty0", "/dev/ttyS0", "/dev/ttyS1"])).toEqual([
      "/dev/ttyS0",
      "/dev/ttyS1",
    ]);
  });

  test("preserves macOS and Windows port names", () => {
    expect(
      filterSelectableSerialPorts([
        "/dev/tty.usbmodem1101",
        "/dev/cu.usbmodem1101",
        "COM3",
      ]),
    ).toEqual(["/dev/tty.usbmodem1101", "/dev/cu.usbmodem1101", "COM3"]);
  });
});
