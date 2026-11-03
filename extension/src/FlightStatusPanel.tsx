import { PanelExtensionContext } from "@lichtblick/suite";
import { CSSProperties, ReactElement, useEffect, useLayoutEffect, useState } from "react";
import { createRoot } from "react-dom/client";

import {
  ALL_TOPICS,
  Field,
  Latest,
  Level,
  Outage,
  applyMessages,
  summarize,
  trackOutage,
} from "./flightStatus";

type Palette = Record<Level | "text" | "muted" | "track", string>;

const PALETTES: Record<"light" | "dark", Palette> = {
  dark: {
    text: "#e6e6e6",
    muted: "#9a9a9a",
    track: "#3a3a3a",
    ok: "#4caf50",
    warn: "#ffb300",
    alert: "#ef5350",
    none: "#9a9a9a",
  },
  light: {
    text: "#1f1f1f",
    muted: "#666666",
    track: "#dddddd",
    ok: "#2e7d32",
    warn: "#b26a00",
    alert: "#c62828",
    none: "#666666",
  },
};

// re-render this often even without messages, so the link indicator notices when data stops
const TICK_MS = 500;

function FlightStatusPanel({ context }: { context: PanelExtensionContext }): ReactElement {
  const [latest, setLatest] = useState<Latest>({});
  const [now, setNow] = useState(() => Date.now());
  const [outage, setOutage] = useState<Outage | undefined>();
  const [colorScheme, setColorScheme] = useState<"light" | "dark">("dark");
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();

  useLayoutEffect(() => {
    context.onRender = (renderState, done) => {
      setRenderDone(() => done);
      if (renderState.colorScheme) {
        setColorScheme(renderState.colorScheme);
      }
      const frame = renderState.currentFrame;
      if (frame && frame.length > 0) {
        const receivedAt = Date.now();
        setLatest((previous) => applyMessages(previous, frame, receivedAt));
        setNow(receivedAt);
      }
    };
    context.watch("currentFrame");
    context.watch("colorScheme");
    context.subscribe(ALL_TOPICS.map((topic) => ({ topic })));
  }, [context]);

  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  useEffect(() => {
    const timer = setInterval(() => {
      setNow(Date.now());
    }, TICK_MS);
    return () => {
      clearInterval(timer);
    };
  }, []);

  useEffect(() => {
    setOutage((previous) => trackOutage(previous, latest, now));
  }, [latest, now]);

  const view = summarize(latest, now, outage);
  const palette = PALETTES[colorScheme];
  // the values are frozen while the link is lost, grey them out so they don't look current
  const color = (field: Field): CSSProperties => ({
    color: view.lost ? palette.muted : palette[field.level],
  });

  const row: CSSProperties = {
    display: "flex",
    justifyContent: "space-between",
    gap: 8,
    padding: "2px 0",
  };
  const label: CSSProperties = { color: palette.muted };
  const section: CSSProperties = {
    borderTop: `1px solid ${palette.track}`,
    paddingTop: 6,
    marginTop: 6,
  };

  const metric = (name: string, field: Field) => (
    <div style={row}>
      <span style={label}>{name}</span>
      <span style={color(field)}>{field.value}</span>
    </div>
  );

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
      {view.lost && (
        <div
          style={{
            background: palette.alert,
            color: "#ffffff",
            fontSize: 16,
            fontWeight: 700,
            textAlign: "center",
            padding: "8px 12px",
            borderRadius: 4,
            marginBottom: 10,
          }}
        >
          ROS link lost, check QGC
        </div>
      )}
      <div style={{ display: "flex", flexWrap: "wrap", gap: 16, fontSize: 16, fontWeight: 600 }}>
        <span style={color(view.armed)}>● {view.armed.value}</span>
        <span>
          <span style={label}>Mode </span>
          <span style={color(view.mode)}>{view.mode.value}</span>
        </span>
        <span>
          <span style={label}>VTOL </span>
          <span style={color(view.vtol)}>{view.vtol.value}</span>
        </span>
      </div>
      {metric("Control", view.control)}

      <div
        style={{
          ...section,
          display: "grid",
          gridTemplateColumns: "repeat(auto-fit, minmax(180px, 1fr))",
          columnGap: 24,
        }}
      >
        {metric(view.altitude.label, view.altitude)}
        {metric("Airspeed", view.airspeed)}
        {metric("Ground speed", view.groundSpeed)}
        {metric("Climb", view.climb)}
        {metric("Battery", view.battery)}
      </div>

      <div style={section}>
        {metric("Mission", view.mission)}
        <div style={{ height: 6, background: palette.track, borderRadius: 3, marginTop: 2 }}>
          <div
            style={{
              height: "100%",
              width: `${String((view.mission.fraction ?? 0) * 100)}%`,
              background: palette.ok,
              borderRadius: 3,
            }}
          />
        </div>
      </div>

      <div style={section}>
        {metric("Failsafe", view.failsafe)}
        <div style={row}>
          <span style={label}>Link</span>
          <span style={{ color: palette[view.link.level] }}>{view.link.value}</span>
        </div>
      </div>
    </div>
  );
}

export function initFlightStatusPanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<FlightStatusPanel context={context} />);
  return () => {
    root.unmount();
  };
}
