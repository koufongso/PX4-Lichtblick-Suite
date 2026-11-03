import { MessageEvent, PanelExtensionContext } from "@lichtblick/suite";
import { CSSProperties, ReactElement, useEffect, useLayoutEffect, useState } from "react";
import { createRoot } from "react-dom/client";

// Lichtblick -> Jetson: true starts the intruder simulation, false stops it.
// The bridge only lets clients publish to /intruder/* (client_topic_whitelist in foxglove_bridge.yaml).
export const INTRUDER_ENABLE_TOPIC = "/intruder/sim_enable";
// Jetson -> Lichtblick (optional): whether the simulation is actually running.
export const INTRUDER_ACTIVE_TOPIC = "/intruder/sim_active";

const BOOL_SCHEMA = "std_msgs/msg/Bool";
// the bridge only sends definitions for topics it has, so give the player this one in case
// nothing on the Jetson uses std_msgs/msg/Bool yet
const BOOL_DATATYPES = new Map([
  [
    BOOL_SCHEMA,
    {
      name: BOOL_SCHEMA,
      definitions: [{ name: "data", type: "bool", isComplex: false, isArray: false }],
    },
  ],
]);

// resend the last command this often, so a Jetson node started (or restarted) later still gets it
const REPEAT_MS = 1000;
// the Jetson feedback counts as lost after this long without a message
const FEEDBACK_TIMEOUT_MS = 3000;

type Palette = Record<"text" | "muted" | "track" | "ok" | "alert" | "button", string>;

const PALETTES: Record<"light" | "dark", Palette> = {
  dark: {
    text: "#e6e6e6",
    muted: "#9a9a9a",
    track: "#3a3a3a",
    ok: "#4caf50",
    alert: "#ef5350",
    button: "#2a2a2a",
  },
  light: {
    text: "#1f1f1f",
    muted: "#666666",
    track: "#dddddd",
    ok: "#2e7d32",
    alert: "#c62828",
    button: "#f2f2f2",
  },
};

function ControlPanel({ context }: { context: PanelExtensionContext }): ReactElement {
  // undefined until the user presses a button, nothing is published before that
  const [command, setCommand] = useState<boolean | undefined>();
  const [lastSent, setLastSent] = useState<number | undefined>();
  const [active, setActive] = useState<{ value: boolean; receivedAt: number } | undefined>();
  const [error, setError] = useState<string | undefined>();
  const [now, setNow] = useState(() => Date.now());
  const [colorScheme, setColorScheme] = useState<"light" | "dark">("dark");
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();

  const canPublish = context.advertise != undefined && context.publish != undefined;

  useLayoutEffect(() => {
    context.onRender = (renderState, done) => {
      setRenderDone(() => done);
      if (renderState.colorScheme) {
        setColorScheme(renderState.colorScheme);
      }
      const frame = renderState.currentFrame as
        readonly MessageEvent<{ data?: boolean }>[] | undefined;
      const last = frame?.filter((event) => event.topic === INTRUDER_ACTIVE_TOPIC).at(-1);
      if (last) {
        setActive({ value: last.message.data === true, receivedAt: Date.now() });
      }
    };
    context.watch("currentFrame");
    context.watch("colorScheme");
    context.subscribe([{ topic: INTRUDER_ACTIVE_TOPIC }]);

    if (!canPublish) {
      return;
    }
    context.advertise?.(INTRUDER_ENABLE_TOPIC, BOOL_SCHEMA, { datatypes: BOOL_DATATYPES });
    return () => {
      context.unadvertise?.(INTRUDER_ENABLE_TOPIC);
    };
  }, [context, canPublish]);

  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  useEffect(() => {
    if (command == undefined) {
      return;
    }
    const send = () => {
      try {
        context.publish?.(INTRUDER_ENABLE_TOPIC, { data: command });
        setLastSent(Date.now());
        setError(undefined);
      } catch (err) {
        setError(err instanceof Error ? err.message : String(err));
      }
    };
    send();
    const timer = setInterval(send, REPEAT_MS);
    return () => {
      clearInterval(timer);
    };
  }, [context, command]);

  useEffect(() => {
    const timer = setInterval(() => {
      setNow(Date.now());
    }, 500);
    return () => {
      clearInterval(timer);
    };
  }, []);

  const palette = PALETTES[colorScheme];
  const row: CSSProperties = {
    display: "flex",
    justifyContent: "space-between",
    gap: 8,
    padding: "2px 0",
  };
  const label: CSSProperties = { color: palette.muted };
  const button = (action: "start" | "stop"): CSSProperties => {
    const color = action === "start" ? palette.ok : palette.alert;
    const selected = command === (action === "start");
    return {
      flex: 1,
      padding: "8px 0",
      fontSize: 14,
      fontWeight: 600,
      color: selected ? "#ffffff" : color,
      background: selected ? color : palette.button,
      border: `1px solid ${color}`,
      borderRadius: 4,
      cursor: canPublish ? "pointer" : "not-allowed",
      opacity: canPublish ? 1 : 0.5,
    };
  };

  const commandText = command == undefined ? "none sent" : command ? "START" : "STOP";
  const sentText =
    lastSent == undefined ? "" : ` (${String(Math.round((now - lastSent) / 100) / 10)} s ago)`;

  const feedbackFresh = active != undefined && now - active.receivedAt < FEEDBACK_TIMEOUT_MS;
  const feedbackText = active == undefined ? "no feedback" : active.value ? "running" : "stopped";
  const feedbackColor = !feedbackFresh ? palette.muted : active.value ? palette.ok : palette.text;

  return (
    <div
      style={{
        padding: 12,
        fontFamily: "sans-serif",
        fontSize: 13,
        color: palette.text,
        height: "100%",
        overflowY: "auto",
      }}
    >
      <div style={{ fontSize: 16, fontWeight: 600, marginBottom: 8 }}>Intruder simulation</div>
      <div style={{ display: "flex", gap: 8 }}>
        <button
          style={button("start")}
          disabled={!canPublish}
          onClick={() => {
            setCommand(true);
          }}
        >
          Start
        </button>
        <button
          style={button("stop")}
          disabled={!canPublish}
          onClick={() => {
            setCommand(false);
          }}
        >
          Stop
        </button>
      </div>

      <div style={{ borderTop: `1px solid ${palette.track}`, paddingTop: 6, marginTop: 10 }}>
        <div style={row}>
          <span style={label}>Command</span>
          <span>
            {commandText}
            <span style={label}>{sentText}</span>
          </span>
        </div>
        <div style={row}>
          <span style={label}>Jetson</span>
          <span style={{ color: feedbackColor }}>
            {feedbackText}
            {active != undefined && !feedbackFresh ? " (stale)" : ""}
          </span>
        </div>
        <div style={{ ...label, fontSize: 11, marginTop: 4 }}>
          {INTRUDER_ENABLE_TOPIC} (std_msgs/Bool), resent every {String(REPEAT_MS / 1000)} s
        </div>
      </div>

      {!canPublish && (
        <div style={{ color: palette.alert, marginTop: 8 }}>
          This data source can not publish. Connect to the Jetson with Foxglove WebSocket.
        </div>
      )}
      {error != undefined && (
        <div style={{ color: palette.alert, marginTop: 8 }}>Publish failed: {error}</div>
      )}
    </div>
  );
}

export function initControlPanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<ControlPanel context={context} />);
  return () => {
    root.unmount();
  };
}
