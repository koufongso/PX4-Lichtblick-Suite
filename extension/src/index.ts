import { ExtensionContext } from "@lichtblick/suite";

import { initFlightMapPanel } from "./FlightMapPanel";
import { initFlightStatusPanel } from "./FlightStatusPanel";
import { registerConverters } from "./converters";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({ name: "Flight Status", initPanel: initFlightStatusPanel });
  extensionContext.registerPanel({ name: "Flight Map", initPanel: initFlightMapPanel });
  registerConverters(extensionContext);
}
