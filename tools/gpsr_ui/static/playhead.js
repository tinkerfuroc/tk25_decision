// tools/gpsr_ui/static/playhead.js
// The single source of time for every panel. Everything -- tree state,
// frame selection, event highlighting -- derives from this one value.
export function createPlayhead({ start, end }) {
  let value = start;
  const subscribers = [];

  const clamp = (v) => {
    if (!Number.isFinite(v)) return start;
    if (v < start) return start;
    if (v > end) return end;
    return v;
  };

  return {
    clamp,
    get: () => value,
    set(next) {
      const clamped = clamp(next);
      if (clamped === value) return;
      value = clamped;
      for (const fn of subscribers) fn(value);
    },
    subscribe(fn) {
      subscribers.push(fn);
      return () => {
        const i = subscribers.indexOf(fn);
        if (i >= 0) subscribers.splice(i, 1);
      };
    },
  };
}
