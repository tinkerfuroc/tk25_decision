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
      // Task 9/10 each subscribe their own panel (tree, camera viewer)
      // alongside the ribbon's own cursor. A bug in one panel's callback
      // must not stop the others from updating, and must not escape set()
      // to whatever triggered it (a click handler, a slider drag) -- that
      // would look like the whole page broke instead of one panel.
      for (const fn of subscribers) {
        try {
          fn(value);
        } catch (err) {
          console.error(
            "playhead subscriber threw; other subscribers still ran",
            fn, err);
        }
      }
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
