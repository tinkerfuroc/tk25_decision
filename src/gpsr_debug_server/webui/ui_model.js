/*
 * Pure data helpers for the GPSR debugger UI.
 *
 * This file deliberately has no DOM, fetch, or storage dependencies so the
 * same rules can be exercised with `node --test` and used directly by the
 * browser.  Values returned from an event remain data; this module never
 * turns telemetry into HTML.
 */
(function attachGpsrUiModel(root, factory) {
  const api = factory();
  if (typeof module !== "undefined" && module.exports) module.exports = api;
  if (root) root.GpsrUiModel = api;
})(typeof globalThis !== "undefined" ? globalThis : this, function createGpsrUiModel() {
  "use strict";

  const COLLAPSIBLE_EVENT_TYPES = new Set([
    "run.heartbeat",
    "heartbeat",
    "tree.node_states_changed",
    "tree.tick_observed",
  ]);
  const DEFAULT_QUERY_STATE = Object.freeze({
    trajectoryId: null,
    at: null,
    view: "overview",
    taskId: null,
    search: "",
    statuses: [],
    categories: [],
    raw: false,
    focusId: null,
    treeRevision: null,
    treeMode: "planned",
  });

  function text(value) {
    return value == null ? "" : String(value);
  }

  function normalizedText(value) {
    return text(value).trim().toLowerCase();
  }

  function eventType(event) {
    if (!event || typeof event !== "object") return "event";
    return text(event.type || event.event_type || "event").trim() || "event";
  }

  function eventPayload(event) {
    if (!event || typeof event !== "object") return {};
    const payload = event.payload == null ? event.data : event.payload;
    return payload && typeof payload === "object" && !Array.isArray(payload) ? payload : {};
  }

  /** Return a stable visual category for a BT/lifecycle status. */
  function statusCategory(value) {
    const status = normalizedText(value).replace(/[\s_-]+/g, " ");
    if (!status || status === "unknown") return "unknown";
    if (/(cancel|cancell?ed|terminated)/.test(status)) return "cancelled";
    if (/(correct|supersed)/.test(status)) return "corrected";
    if (/(fail|error|abort|reject|denied|timeout)/.test(status)) return "failure";
    if (/(success|succeed|complete|finish|done|pass)/.test(status)) return "success";
    if (/(running|active|execut|progress|pending|start|queued)/.test(status)) return "running";
    if (/(invalid|not visited|unvisited|idle|none)/.test(status)) return "invalid";
    return "unknown";
  }

  /* Name used by the original UI's CSS. Unknown/invalid values get no class. */
  function statusClass(value) {
    const category = statusCategory(value);
    return ["failure", "running", "success", "cancelled", "corrected"].includes(category) ? category : "";
  }

  /** Classify an event without relying on producer-specific payload layouts. */
  function eventCategory(event) {
    const type = eventType(event).toLowerCase();
    if (type === "heartbeat" || type.endsWith(".heartbeat")) return "heartbeat";
    if (/(fail|error|reject|warning|warn)/.test(type)) return "failure";
    if (type.startsWith("tree.")) return "tree";
    if (type.startsWith("planner.") || type.startsWith("plan.") || type.startsWith("supervisor.")) return "planning";
    if (type.startsWith("step.") || type.includes("execution")) return "execution";
    if (type.startsWith("run.") || type.startsWith("mission.") || type.startsWith("trajectory.")) return "lifecycle";
    if (type.startsWith("agent.") || type.startsWith("proposal.") || type.startsWith("vote.")) return "agent";
    if (type.startsWith("state.") || type.startsWith("blackboard.")) return "state";
    if (type.startsWith("intervention.") || type.startsWith("control.")) return "intervention";
    return "event";
  }

  function eventStatus(event) {
    if (!event || typeof event !== "object") return "";
    const payload = eventPayload(event);
    return text(event.status || event.outcome || payload.status || payload.outcome || payload.result || "");
  }

  function eventTimeMs(event) {
    if (!event || typeof event !== "object") return null;
    const candidate = event.occurred_at ?? event.timestamp ?? event.time ?? event.created_at;
    if (candidate instanceof Date) {
      const value = candidate.getTime();
      return Number.isFinite(value) ? value : null;
    }
    if (typeof candidate === "number" && Number.isFinite(candidate)) {
      // Unix timestamps from the stdlib client may be expressed in seconds.
      return Math.abs(candidate) < 100000000000 ? candidate * 1000 : candidate;
    }
    if (typeof candidate === "string" && candidate.trim()) {
      const value = Date.parse(candidate);
      return Number.isFinite(value) ? value : null;
    }
    return null;
  }

  function isCollapsibleEvent(event) {
    return COLLAPSIBLE_EVENT_TYPES.has(eventType(event).toLowerCase());
  }

  /**
   * Collapse only adjacent high-rate events of the same type.  A malformed or
   * backwards timestamp never causes records to disappear into a group.
   */
  function groupEvents(events, options) {
    const source = Array.isArray(events) ? events : [];
    const settings = options && typeof options === "object" ? options : {};
    const raw = settings.raw === true;
    const windowMs = Number.isFinite(Number(settings.windowMs)) && Number(settings.windowMs) >= 0
      ? Number(settings.windowMs)
      : 1000;
    const groups = [];

    for (const event of source) {
      const type = eventType(event);
      const time = eventTimeMs(event);
      const previous = groups[groups.length - 1];
      const canJoin = !raw && previous && previous.type === type && isCollapsibleEvent(event)
        && previous.lastTimeMs != null && time != null && time >= previous.lastTimeMs
        && time - previous.lastTimeMs <= windowMs;
      if (canJoin) {
        previous.events.push(event);
        previous.count += 1;
        previous.last = event;
        previous.lastTimeMs = time;
        previous.lastOccurredAt = event && typeof event === "object"
          ? (event.occurred_at ?? event.timestamp ?? event.time ?? event.created_at ?? null)
          : null;
        previous.collapsed = true;
      } else {
        groups.push({
          type,
          category: eventCategory(event),
          event,
          events: [event],
          first: event,
          last: event,
          count: 1,
          collapsed: false,
          firstTimeMs: time,
          lastTimeMs: time,
          firstOccurredAt: event && typeof event === "object"
            ? (event.occurred_at ?? event.timestamp ?? event.time ?? event.created_at ?? null)
            : null,
          lastOccurredAt: event && typeof event === "object"
            ? (event.occurred_at ?? event.timestamp ?? event.time ?? event.created_at ?? null)
            : null,
        });
      }
    }
    return groups;
  }

  function asTextSet(value) {
    const values = value == null ? [] : (Array.isArray(value) || value instanceof Set ? Array.from(value) : [value]);
    return new Set(values.map(normalizedText).filter(Boolean));
  }

  function eventAgentId(event) {
    const payload = eventPayload(event);
    return text((event && (event.agent_id || event.agentId)) || payload.agent_id || payload.agentId || "");
  }

  function eventTaskId(event) {
    const payload = eventPayload(event);
    return text((event && (event.task_id || event.taskId)) || payload.task_id || payload.taskId || "");
  }

  function safeSearchText(value, seen, depth) {
    const currentDepth = depth || 0;
    if (value == null) return "";
    if (typeof value === "string" || typeof value === "number" || typeof value === "boolean") return text(value);
    if (currentDepth >= 5 || typeof value !== "object") return "";
    const known = seen || new WeakSet();
    if (known.has(value)) return "[circular]";
    known.add(value);
    const parts = Array.isArray(value)
      ? value.slice(0, 100).map(item => safeSearchText(item, known, currentDepth + 1))
      : Object.keys(value).sort().slice(0, 100).flatMap(key => [key, safeSearchText(value[key], known, currentDepth + 1)]);
    known.delete(value);
    return parts.join(" ");
  }

  /** A searchable, lower-cased text representation which tolerates circular payloads. */
  function eventSearchText(event) {
    return safeSearchText(event).toLowerCase();
  }

  function matchesEvent(event, filters) {
    const options = filters && typeof filters === "object" ? filters : {};
    const types = asTextSet(options.types ?? options.type);
    const categories = asTextSet(options.categories ?? options.category);
    const statuses = asTextSet(options.statuses ?? options.status);
    const phases = asTextSet(options.phases ?? options.phase);
    const taskIds = asTextSet(options.taskIds ?? options.taskId);
    const agentIds = asTextSet(options.agentIds ?? options.agentId);
    const type = normalizedText(eventType(event));
    const category = eventCategory(event);
    const status = normalizedText(eventStatus(event));
    const phase = normalizedText(event && event.phase);
    const taskId = normalizedText(eventTaskId(event));
    const agentId = normalizedText(eventAgentId(event));
    const time = eventTimeMs(event);
    const from = queryTime(options.from);
    const to = queryTime(options.to);
    const search = normalizedText(options.search ?? options.q);

    if (types.size && !types.has(type)) return false;
    if (categories.size && !categories.has(category)) return false;
    if (statuses.size && !statuses.has(status) && !statuses.has(statusCategory(status))) return false;
    if (phases.size && !phases.has(phase)) return false;
    if (taskIds.size && !taskIds.has(taskId)) return false;
    if (agentIds.size && !agentIds.has(agentId)) return false;
    if (from != null && (time == null || time < from)) return false;
    if (to != null && (time == null || time > to)) return false;
    if (search && !eventSearchText(event).includes(search)) return false;
    return true;
  }

  function filterEvents(events, filters) {
    return (Array.isArray(events) ? events : []).filter(event => matchesEvent(event, filters));
  }

  /**
   * Fold the append-only supervisor stream into one visual record per BT
   * checkpoint. Unknown supervisor events stay attached to the record so a
   * newer producer remains inspectable in an older dashboard.
   */
  function supervisorCheckpoints(events) {
    const checkpoints = new Map();
    for (const event of Array.isArray(events) ? events : []) {
      const type = eventType(event).toLowerCase();
      if (!type.startsWith("supervisor.")) continue;
      const payload = eventPayload(event);
      const checkpointId = text(payload.checkpoint_id || event.checkpoint_id).trim();
      if (!checkpointId) continue;
      if (!checkpoints.has(checkpointId)) {
        checkpoints.set(checkpointId, {
          checkpointId,
          sequence: Number.isFinite(Number(event.sequence)) ? Number(event.sequence) : 0,
          created: null,
          verdict: null,
          queries: [],
          recoveries: [],
          global: null,
          unavailable: null,
          events: [],
        });
      }
      const checkpoint = checkpoints.get(checkpointId);
      checkpoint.sequence = Math.min(
        checkpoint.sequence,
        Number.isFinite(Number(event.sequence)) ? Number(event.sequence) : checkpoint.sequence,
      );
      checkpoint.events.push(event);
      if (type === "supervisor.checkpoint.created") checkpoint.created = payload;
      else if (type === "supervisor.verdict.received") checkpoint.verdict = payload;
      else if (type === "supervisor.query.completed" || type === "supervisor.query.failed") {
        checkpoint.queries.push(payload);
      } else if (type.startsWith("supervisor.recovery.")) {
        checkpoint.recoveries.push({ type, sequence: event.sequence, ...payload });
      } else if (type === "supervisor.global.proposed") checkpoint.global = payload;
      else if (type === "supervisor.unavailable") checkpoint.unavailable = payload;
    }
    return Array.from(checkpoints.values()).sort((left, right) => left.sequence - right.sequence);
  }

  function queryTime(value) {
    if (value == null || value === "") return null;
    if (typeof value === "number" && Number.isFinite(value)) return Math.abs(value) < 100000000000 ? value * 1000 : value;
    if (value instanceof Date) return Number.isFinite(value.getTime()) ? value.getTime() : null;
    const parsed = Date.parse(String(value));
    return Number.isFinite(parsed) ? parsed : null;
  }

  /** Return a readable, bounded identifier without losing its distinguishing suffix. */
  function shortTrajectoryId(value, maxLength) {
    const raw = typeof value === "object" && value !== null
      ? text(value.trajectory_id || value.run_id || value.id)
      : text(value);
    const id = raw.trim();
    const maximum = Number.isFinite(Number(maxLength)) ? Math.max(5, Math.floor(Number(maxLength))) : 24;
    if (id.length <= maximum) return id;
    const suffixLength = Math.max(2, Math.floor((maximum - 1) / 2));
    const prefixLength = maximum - suffixLength - 1;
    return `${id.slice(0, prefixLength)}…${id.slice(-suffixLength)}`;
  }

  function nodeId(value) {
    if (value == null) return "";
    if (typeof value === "object") return text(value.id ?? value.node_id ?? value.nodeId ?? value.uid).trim();
    return text(value).trim();
  }

  function objectEntries(value) {
    return value && typeof value === "object" && !Array.isArray(value) ? Object.entries(value) : [];
  }

  function childReferences(node) {
    if (!node || typeof node !== "object") return [];
    const raw = node.children ?? node.child_ids ?? node.childIds ?? [];
    return Array.isArray(raw) ? raw.map(nodeId).filter(Boolean) : [];
  }

  function edgeEndpoints(edge) {
    if (Array.isArray(edge)) return [nodeId(edge[0]), nodeId(edge[1])];
    if (!edge || typeof edge !== "object") return ["", ""];
    return [nodeId(edge.from ?? edge.source ?? edge.parent ?? edge.parent_id ?? edge.parentId),
      nodeId(edge.to ?? edge.target ?? edge.child ?? edge.child_id ?? edge.childId)];
  }

  function plainMap() {
    return Object.create(null);
  }

  function booleanOrNull(value) {
    if (value === true || value === false) return value;
    const lowered = normalizedText(value);
    if (["true", "1", "yes", "on"].includes(lowered)) return true;
    if (["false", "0", "no", "off"].includes(lowered)) return false;
    return null;
  }

  function firstDefined(source, paths) {
    for (const path of paths) {
      let value = source;
      for (const part of path) {
        if (!value || typeof value !== "object" || !(part in value)) {
          value = undefined;
          break;
        }
        value = value[part];
      }
      if (value !== undefined) return value;
    }
    return undefined;
  }

  function canonicalNodeClass(value) {
    const kind = normalizedText(value).replace(/[\s_-]+/g, "");
    if (!kind) return "";
    if (/(^|\.)(sequence|sequencestar)$/.test(kind) || kind.includes("sequence")) return "sequence";
    if (/(selector|fallback|choice|priority)/.test(kind)) return "selector";
    if (kind.includes("parallel")) return "parallel";
    if (/(decorator|inverter|oneshot|retry|timeout|ratelimit|failureis|successis|runningis|statusto)/.test(kind)) return "decorator";
    if (/(^|\.)(condition|action|behaviour|behavior|leaf)$/.test(kind) || /(^|\.)(condition|action)$/.test(kind)) return "leaf";
    if (kind.includes("composite")) return "composite";
    if (kind === "root") return "root";
    return "";
  }

  function concreteNodeKind(value) {
    const raw = normalizedText(value);
    const compact = raw.replace(/[\s_.-]+/g, "");
    if (!compact) return "";
    if (compact.includes("sequence")) return "sequence";
    if (/(selector|fallback|choice|priority)/.test(compact)) return "selector";
    if (compact.includes("parallel")) return "parallel";
    if (compact.includes("retry")) return "retry";
    if (compact.includes("repeat")) return "repeat";
    if (compact.includes("timeout")) return "timeout";
    if (compact.includes("oneshot")) return "one_shot";
    if (compact.includes("foreach")) return "for_each";
    if (compact.includes("inverter")) return "inverter";
    const statusMap = [
      ["failureissuccess", "failure_is_success"],
      ["failureisrunning", "failure_is_running"],
      ["successisfailure", "success_is_failure"],
      ["successisrunning", "success_is_running"],
      ["runningisfailure", "running_is_failure"],
      ["runningissuccess", "running_is_success"],
    ].find(([needle]) => compact.includes(needle));
    if (statusMap) return statusMap[1];
    if (["leaf", "composite", "decorator", "root"].includes(compact)) return compact;
    return canonicalNodeClass(value);
  }

  function copiedModelValue(value, depth, seen) {
    const currentDepth = depth || 0;
    if (value == null || typeof value === "string" || typeof value === "number" || typeof value === "boolean") return value;
    if (currentDepth >= 5 || typeof value !== "object") return text(value);
    const known = seen || new WeakSet();
    if (known.has(value)) return "[circular]";
    known.add(value);
    const copy = Array.isArray(value)
      ? value.slice(0, 100).map(item => copiedModelValue(item, currentDepth + 1, known))
      : Object.keys(value).slice(0, 100).reduce((result, key) => {
        result[key] = copiedModelValue(value[key], currentDepth + 1, known);
        return result;
      }, {});
    known.delete(value);
    return copy;
  }

  function nodeIdList(value) {
    const values = value == null ? [] : (Array.isArray(value) || value instanceof Set ? Array.from(value) : [value]);
    return Array.from(new Set(values.map(nodeId).filter(Boolean)));
  }

  /**
   * Infer control-flow semantics from either the future ``node_class`` field
   * or the legacy ``type``/``node_type`` class-name payload.  The returned DTO
   * intentionally has snake- and camel-case properties so old browser code
   * can consume it without normalising producer versions.
   */
  function nodeSemantics(node) {
    const source = node && typeof node === "object" ? node : {};
    const explicit = firstDefined(source, [["node_class"], ["nodeClass"]]);
    const type = firstDefined(source, [["type"], ["node_type"], ["nodeType"], ["class_name"], ["className"], ["class"]]);
    const producerKind = firstDefined(source, [
      ["semantics", "kind"], ["semantics", "control_flow"], ["semantics", "controlFlow"],
      ["kind"], ["control_flow"], ["controlFlow"],
    ]);
    const explicitClass = canonicalNodeClass(explicit);
    const typeClass = canonicalNodeClass(type);
    const kindClass = concreteNodeKind(producerKind);
    const typeKind = concreteNodeKind(type);
    const hasChildren = childReferences(source).length > 0;
    // Newer producers carry a generic category in node_class (for example
    // "composite") and the concrete control node in semantics.kind. Prefer the
    // latter, while accepting complete legacy type names unchanged.
    const nodeClass = kindClass || typeKind || explicitClass || typeClass || (hasChildren ? "composite" : "leaf");
    const memoryValue = firstDefined(source, [
      ["semantics", "memory"], ["memory"], ["is_memory"], ["isMemory"], ["policy", "memory"], ["metadata", "memory"],
    ]);
    const memory = booleanOrNull(memoryValue);
    const synchronise = booleanOrNull(firstDefined(source, [
      ["semantics", "synchronise"], ["semantics", "synchronize"], ["synchronise"], ["synchronize"],
    ]));
    const selectedChildIds = nodeIdList(firstDefined(source, [
      ["semantics", "selected_child_ids"], ["semantics", "selectedChildIds"], ["selected_child_ids"], ["selectedChildIds"],
    ]));
    const successPolicy = firstDefined(source, [
      ["semantics", "parallel_policy"], ["semantics", "parallelPolicy"],
      ["semantics", "success_policy"], ["semantics", "successPolicy"], ["success_policy"], ["successPolicy"],
      ["parallel_policy"], ["parallelPolicy"],
    ]);
    const counters = firstDefined(source, [["semantics", "counters"], ["counters"]]);
    const rawNodeClass = text(firstDefined(source, [["raw_node_class"], ["rawNodeClass"]]) ?? explicit).trim() || null;
    const producerCategory = canonicalNodeClass(firstDefined(source, [["semantics", "category"], ["category"]]));
    const category = producerCategory === "decorator" || producerCategory === "leaf" || producerCategory === "composite"
      ? producerCategory
      : ["sequence", "selector", "parallel", "composite", "root"].includes(nodeClass)
        ? "composite"
        : nodeClass === "leaf"
          ? "leaf"
          : "decorator";
    const rawKind = text(firstDefined(source, [["semantics", "kind"], ["kind"]]) ?? "").trim() || null;
    const controlFlow = firstDefined(source, [["semantics", "control_flow"], ["semantics", "controlFlow"], ["control_flow"], ["controlFlow"]]);
    const isComposite = ["sequence", "selector", "parallel", "composite", "root"].includes(nodeClass) || hasChildren;
    const isControlFlow = ["sequence", "selector", "parallel", "decorator", "composite", "root"].includes(nodeClass);
    return {
      node_class: nodeClass,
      nodeClass,
      kind: nodeClass,
      raw_kind: rawKind,
      rawKind,
      category,
      raw_node_class: rawNodeClass,
      rawNodeClass,
      control_flow: controlFlow == null ? null : copiedModelValue(controlFlow),
      controlFlow: controlFlow == null ? null : copiedModelValue(controlFlow),
      memory,
      success_policy: successPolicy == null ? null : copiedModelValue(successPolicy),
      successPolicy: successPolicy == null ? null : copiedModelValue(successPolicy),
      synchronise,
      selected_child_ids: selectedChildIds,
      selectedChildIds: selectedChildIds.slice(),
      counters: counters == null ? null : copiedModelValue(counters),
      is_composite: isComposite,
      isComposite,
      is_control_flow: isControlFlow,
      isControlFlow,
      has_children: hasChildren,
      hasChildren,
      source: kindClass ? "semantics.kind" : (explicitClass ? "node_class" : (typeKind || typeClass ? "type" : "shape")),
    };
  }

  /**
   * Normalise array/object node documents and explicit edges into an immutable
   * graph-like DTO. Bad references and cycles are retained as warnings instead
   * of being followed recursively.
   */
  function normalizeTree(document) {
    const source = document && typeof document === "object" ? document : {};
    const rawNodes = Array.isArray(source) ? source : source.nodes;
    const entries = Array.isArray(rawNodes)
      ? rawNodes.map((node, index) => [String(index), node])
      : objectEntries(rawNodes);
    const nodes = [];
    const byId = plainMap();
    const rawIdMap = plainMap();
    const warnings = [];

    entries.forEach(([key, rawNode], index) => {
      const sourceNode = rawNode && typeof rawNode === "object" && !Array.isArray(rawNode) ? rawNode : {};
      const originalId = nodeId(sourceNode) || text(key).trim() || `node-${index + 1}`;
      let id = originalId;
      let duplicate = 2;
      while (Object.prototype.hasOwnProperty.call(byId, id)) id = `${originalId}#${duplicate++}`;
      if (id !== originalId) warnings.push(`duplicate node id: ${originalId}`);
      if (!Object.prototype.hasOwnProperty.call(rawIdMap, originalId)) rawIdMap[originalId] = id;
      const semantics = nodeSemantics(sourceNode);
      const copy = { ...sourceNode, id, node_class: semantics.node_class, children: [] };
      if (sourceNode.node_class != null && copy.raw_node_class == null && copy.rawNodeClass == null) {
        copy.raw_node_class = sourceNode.node_class;
      }
      delete copy.node_id;
      delete copy.nodeId;
      nodes.push(copy);
      byId[id] = copy;
    });

    const childrenById = plainMap();
    const parentsById = plainMap();
    nodes.forEach(node => {
      childrenById[node.id] = [];
      parentsById[node.id] = [];
    });
    const resolveId = reference => rawIdMap[reference] || (byId[reference] ? reference : "");
    const createsCycle = (parentId, childId) => {
      const pending = [childId];
      const seen = new Set();
      while (pending.length) {
        const current = pending.pop();
        if (current === parentId) return true;
        if (seen.has(current)) continue;
        seen.add(current);
        (childrenById[current] || []).forEach(next => pending.push(next));
      }
      return false;
    };
    const link = (parentReference, childReference) => {
      const parentId = resolveId(parentReference);
      const childId = resolveId(childReference);
      if (!parentId || !childId) {
        warnings.push(`unknown tree edge: ${text(parentReference)} -> ${text(childReference)}`);
        return;
      }
      if (parentId === childId) {
        warnings.push(`self-referential tree edge: ${parentId}`);
        return;
      }
      if (createsCycle(parentId, childId)) {
        warnings.push(`cyclic tree edge: ${parentId} -> ${childId}`);
        return;
      }
      if (!childrenById[parentId].includes(childId)) childrenById[parentId].push(childId);
      if (!parentsById[childId].includes(parentId)) parentsById[childId].push(parentId);
    };

    entries.forEach(([key, rawNode], index) => {
      const raw = rawNode && typeof rawNode === "object" ? rawNode : {};
      const parent = nodes[index];
      childReferences(raw).forEach(child => link(parent.id, child));
      const parentReference = nodeId(raw.parent_id ?? raw.parentId ?? raw.parent);
      if (parentReference) link(parentReference, parent.id);
    });
    const edges = Array.isArray(source.edges) ? source.edges : [];
    edges.forEach(edge => {
      const [parentId, childId] = edgeEndpoints(edge);
      if (parentId || childId) link(parentId, childId);
    });

    nodes.forEach(node => {
      node.children = childrenById[node.id].slice();
      node.parentId = parentsById[node.id][0] || null;
      node.parents = parentsById[node.id].slice();
    });
    const declaredRoots = Array.isArray(source.roots)
      ? source.roots
      : [source.root_id ?? source.rootId ?? source.root].filter(value => value != null);
    const roots = declaredRoots.map(nodeId).map(resolveId).filter(Boolean);
    const inferredRoots = nodes.filter(node => parentsById[node.id].length === 0).map(node => node.id);
    const rootIds = Array.from(new Set((roots.length ? roots : inferredRoots).filter(id => byId[id])));
    if (!rootIds.length && nodes.length) {
      rootIds.push(nodes[0].id);
      warnings.push("tree has no root; first node was used as the root");
    }

    return { nodes, byId, childrenById, parentsById, rootIds, roots: rootIds.map(id => byId[id]), warnings };
  }

  function normalizedTree(tree) {
    return tree && Array.isArray(tree.nodes) && tree.byId && tree.childrenById && tree.parentsById ? tree : normalizeTree(tree);
  }

  function resolveTreeId(tree, id) {
    const graph = normalizedTree(tree);
    const requested = nodeId(id);
    return graph.byId[requested] ? requested : "";
  }

  /** Ancestors are ordered root-first and do not include the requested node. */
  function ancestorIds(tree, id) {
    const graph = normalizedTree(tree);
    let current = resolveTreeId(graph, id);
    if (!current) return [];
    const result = [];
    const seen = new Set([current]);
    while (graph.parentsById[current] && graph.parentsById[current].length) {
      const parent = graph.parentsById[current][0];
      if (seen.has(parent)) break;
      result.unshift(parent);
      seen.add(parent);
      current = parent;
    }
    return result;
  }

  function pathToNode(tree, id) {
    const target = resolveTreeId(tree, id);
    return target ? ancestorIds(tree, target).concat(target) : [];
  }

  /** A finite pre-order traversal, safe for malformed cyclic graph documents. */
  function subtreeIds(tree, id, options) {
    const graph = normalizedTree(tree);
    const root = resolveTreeId(graph, id);
    if (!root) return [];
    const settings = options && typeof options === "object" ? options : {};
    const includeRoot = settings.includeRoot !== false;
    const maxNodes = Number.isFinite(Number(settings.maxNodes)) ? Math.max(0, Math.floor(Number(settings.maxNodes))) : Infinity;
    const result = [];
    const seen = new Set();
    const visit = nodeId => {
      if (!nodeId || seen.has(nodeId) || result.length >= maxNodes) return;
      seen.add(nodeId);
      if (includeRoot || nodeId !== root) result.push(nodeId);
      (graph.childrenById[nodeId] || []).forEach(visit);
    };
    visit(root);
    return result;
  }

  function usableIdSet(value) {
    const values = value == null ? [] : (Array.isArray(value) || value instanceof Set ? Array.from(value) : [value]);
    return new Set(values.map(nodeId).filter(Boolean));
  }

  /**
   * Return the ids needed to render a large executor tree. A focus keeps its
   * complete ancestor path and subtree visible; without focus, expansion is
   * honoured when supplied (or the whole tree is returned by default).
   */
  function treeVisibility(tree, options) {
    const graph = normalizedTree(tree);
    const settings = options && typeof options === "object" ? options : {};
    const focusId = resolveTreeId(graph, settings.focusId ?? settings.activeId ?? settings.nodeId);
    const maxNodes = Number.isFinite(Number(settings.maxNodes)) ? Math.max(0, Math.floor(Number(settings.maxNodes))) : Infinity;
    const expanded = usableIdSet(settings.expandedIds ?? settings.expanded);
    const restrictExpansion = settings.expandedIds != null || settings.expanded != null;
    const pathIds = focusId ? pathToNode(graph, focusId) : [];
    const focusedSubtree = focusId ? subtreeIds(graph, focusId, { maxNodes }) : [];
    const wanted = focusId ? new Set(pathIds.concat(focusedSubtree)) : null;
    const visibleIds = [];
    const seen = new Set();
    const visit = id => {
      if (!id || seen.has(id) || visibleIds.length >= maxNodes) return;
      seen.add(id);
      if (!wanted || wanted.has(id)) visibleIds.push(id);
      if (wanted && !wanted.has(id)) return;
      const mayExpand = focusId ? true : (!restrictExpansion || expanded.has(id));
      if (!mayExpand) return;
      (graph.childrenById[id] || []).forEach(visit);
    };
    graph.rootIds.forEach(visit);
    // A cyclic/disconnected malformed document still gets a bounded render.
    const structurallyReachable = new Set();
    const markReachable = id => {
      if (!id || structurallyReachable.has(id)) return;
      structurallyReachable.add(id);
      (graph.childrenById[id] || []).forEach(markReachable);
    };
    graph.rootIds.forEach(markReachable);
    graph.nodes.forEach(node => {
      if (!structurallyReachable.has(node.id) && visibleIds.length < maxNodes && (!wanted || wanted.has(node.id))) visit(node.id);
    });
    return {
      visibleIds,
      pathIds,
      ancestorIds: focusId ? pathIds.slice(0, -1) : [],
      subtreeIds: focusedSubtree,
      focusId: focusId || null,
      truncated: visibleIds.length >= maxNodes && graph.nodes.length > visibleIds.length,
    };
  }

  function visibleTreeIds(tree, options) {
    return treeVisibility(tree, options).visibleIds;
  }

  function graphNode(graph, value) {
    const model = normalizedTree(graph);
    const id = resolveTreeId(model, value);
    return id ? model.byId[id] : null;
  }

  function childIndex(parent, childId, graph) {
    const model = normalizedTree(graph);
    const parentNode = graphNode(model, parent);
    if (!parentNode) return -1;
    return (model.childrenById[parentNode.id] || []).indexOf(resolveTreeId(model, childId));
  }

  /**
   * Explain the normal control-flow condition for a parent -> child edge.
   * The label is static by design; current runtime evidence belongs to
   * ``activationState`` and is added to semantic-skeleton edge DTOs.
   */
  function edgeActivationLabel(parent, childId, graph) {
    const model = normalizedTree(graph);
    const parentNode = graphNode(model, parent);
    if (!parentNode) return "tick child";
    const semantics = nodeSemantics(parentNode);
    const index = childIndex(parentNode, childId, model);
    if (semantics.node_class === "sequence") return index <= 0 ? "start sequence" : "after previous succeeds";
    if (semantics.node_class === "selector") return index <= 0 ? "try first branch" : "if earlier branches fail";
    if (semantics.node_class === "parallel") {
      const selected = semantics.selected_child_ids;
      const childIsSelected = selected.includes(resolveTreeId(model, childId));
      if (selected.length) return childIsSelected ? "selected for parallel tick" : "not selected by synchronised parallel";
      if (semantics.synchronise === true) return "tick until synchronised success";
      const policy = normalizedText(semantics.success_policy);
      if (policy.includes("one")) return "tick with siblings (first success wins)";
      if (policy.includes("all")) return "tick with siblings (all must succeed)";
      return "tick with siblings";
    }
    if (semantics.category === "decorator" || semantics.node_class === "decorator") return "pass through decorator";
    if (semantics.node_class === "composite" || semantics.node_class === "root") return "tick child";
    return "child";
  }

  function own(value, key) {
    return Object.prototype.hasOwnProperty.call(value, key);
  }

  function visitRecordId(record, fallbackId) {
    const direct = nodeId(record);
    return direct || nodeId(fallbackId);
  }

  function visitationFrom(value) {
    const result = { available: value != null, ids: new Set(), records: plainMap() };
    const add = (record, fallbackId) => {
      if (record === false || record == null) return;
      if (record && typeof record === "object" && (record.visited === false || record.ticked === false || record.recorded === false)) return;
      const id = visitRecordId(record, fallbackId);
      if (!id) return;
      result.ids.add(id);
      result.records[id] = record && typeof record === "object" ? record : { id };
    };
    const addCollection = collection => {
      if (collection == null) return;
      if (collection instanceof Set || Array.isArray(collection)) {
        Array.from(collection).forEach(item => add(item));
      } else if (typeof collection === "object") {
        if (nodeId(collection)) {
          add(collection);
        } else {
          Object.entries(collection).forEach(([id, record]) => add(record, id));
        }
      } else {
        add(collection);
      }
    };
    if (value == null) return result;
    if (value instanceof Set || Array.isArray(value) || typeof value !== "object") {
      addCollection(value);
      return result;
    }
    const listKeys = [
      "visitedIds", "visited_ids", "tickedIds", "ticked_ids", "activeIds", "active_ids",
      "nodeIds", "node_ids", "visited", "ticked", "active", "nodes", "nodeStates", "node_states", "byId", "by_id",
    ];
    const found = listKeys.filter(key => own(value, key) && typeof value[key] !== "boolean");
    if (found.length) {
      found.forEach(key => addCollection(value[key]));
      return result;
    }
    if (nodeId(value)) {
      add(value);
      return result;
    }
    Object.entries(value).forEach(([id, record]) => add(record, id));
    return result;
  }

  function tickSource(runtime, phase) {
    const source = runtime && typeof runtime === "object" ? runtime : null;
    if (!source) return null;
    const keys = phase === "previous"
      ? ["previousTick", "previous_tick", "previous", "priorTick", "prior_tick", "prior", "lastTick", "last_tick"]
      : ["currentTick", "current_tick", "current", "tick", "currentVisitation", "current_visitation"];
    for (const key of keys) if (own(source, key)) return source[key];
    const directKeys = phase === "previous"
      ? ["previousVisitedIds", "previous_visited_ids", "priorVisitedIds", "prior_visited_ids", "previousNodes", "previous_nodes"]
      : ["currentVisitedIds", "current_visited_ids", "visitedIds", "visited_ids", "tickedIds", "ticked_ids", "activeIds", "active_ids", "nodes"];
    const direct = {};
    let found = false;
    directKeys.forEach(key => {
      if (!own(source, key)) return;
      const lowered = key.toLowerCase();
      if (lowered.includes("node")) direct.nodes = source[key];
      else direct.visitedIds = source[key];
      found = true;
    });
    if (found) return direct;
    // A bare tick record (`{nodes: [...]}`) is also a useful current input.
    if (phase === "current" && (own(source, "nodeStates") || own(source, "node_states") || nodeId(source))) return source;
    return null;
  }

  function runtimeVisitations(runtime) {
    return {
      current: visitationFrom(tickSource(runtime, "current")),
      previous: visitationFrom(tickSource(runtime, "previous")),
    };
  }

  function recordStatus(record) {
    if (!record || typeof record !== "object") return "";
    return text(record.status ?? record.state ?? record.outcome ?? record.result ?? "");
  }

  function runtimeStatus(graph, id, visits) {
    const model = normalizedTree(graph);
    const current = visits.current.records[id];
    const previous = visits.previous.records[id];
    return recordStatus(current) || recordStatus(previous) || text(model.byId[id] && model.byId[id].status);
  }

  function siblingName(graph, id) {
    const model = normalizedTree(graph);
    const node = model.byId[id];
    return text((node && (node.name || node.label || node.id || node.type || node.node_type)) || id || "a previous sibling");
  }

  function activationEvidence(graph, requestedId, runtime, suppliedVisits) {
    const model = normalizedTree(graph);
    const id = resolveTreeId(model, requestedId);
    if (!id) return { state: "not-recorded", reason: "This node is not present in the selected tree." };
    const visits = suppliedVisits || runtimeVisitations(runtime);
    if (visits.current.ids.has(id)) {
      if (visits.previous.ids.has(id)) {
        return { state: "resumed", reason: "Visited on this tick and the preceding tick, so its work continued." };
      }
      return { state: "ticked", reason: "Visited on this tick." };
    }
    if (!visits.current.available) {
      return { state: "not-recorded", reason: "No current tick visitation was recorded." };
    }
    const parentId = (model.parentsById[id] || [])[0];
    if (!parentId) {
      return { state: "not-recorded", reason: "The root was not recorded on this tick." };
    }
    const parent = model.byId[parentId];
    const semantics = nodeSemantics(parent);
    const nearestUnvisitedAncestor = [parentId].concat(ancestorIds(model, parentId).reverse())
      .find(ancestorId => !visits.current.ids.has(ancestorId));
    if (nearestUnvisitedAncestor) {
      return {
        state: "blocked",
        reason: `Blocked because ${siblingName(model, nearestUnvisitedAncestor)} was not visited on this tick.`,
      };
    }
    const siblings = model.childrenById[parentId] || [];
    const index = siblings.indexOf(id);
    const currentSiblingIndices = siblings.map((siblingId, siblingIndex) => ({ id: siblingId, index: siblingIndex }))
      .filter(item => visits.current.ids.has(item.id));
    const earlier = currentSiblingIndices.filter(item => item.index < index);
    const later = currentSiblingIndices.filter(item => item.index > index);
    const firstEarlier = earlier[0];
    const firstLater = later[0];
    const siblingState = item => statusCategory(runtimeStatus(model, item.id, visits));

    if (semantics.node_class === "sequence" && firstEarlier) {
      const blocker = earlier.find(item => ["running", "failure", "cancelled"].includes(siblingState(item)));
      const state = blocker && siblingState(blocker);
      if (state === "running") {
        return { state: "blocked", reason: `Blocked while sequence waits for ${siblingName(model, blocker.id)} to finish.` };
      }
      if (["failure", "cancelled"].includes(state)) {
        return { state: "skipped", reason: `Skipped because sequence stopped after ${siblingName(model, blocker.id)} did not succeed.` };
      }
    }
    if (semantics.node_class === "selector" && firstEarlier) {
      const blocker = earlier.find(item => ["running", "success"].includes(siblingState(item)));
      const state = blocker && siblingState(blocker);
      if (["running", "success"].includes(state)) {
        const detail = state === "running" ? "is still running" : "succeeded";
        return { state: "skipped", reason: `Skipped because selector kept ${siblingName(model, blocker.id)}, which ${detail}.` };
      }
    }
    if (semantics.memory === true && firstLater) {
      if (semantics.node_class === "sequence") {
        return { state: "skipped", reason: `Skipped because memory sequence resumed at ${siblingName(model, firstLater.id)}.` };
      }
      if (semantics.node_class === "selector") {
        return { state: "skipped", reason: `Skipped because memory selector resumed at ${siblingName(model, firstLater.id)}.` };
      }
    }
    if (semantics.node_class === "parallel" && visits.current.ids.has(parentId)) {
      const childStatus = statusCategory(runtimeStatus(model, id, visits));
      const childWasSelected = semantics.selected_child_ids.includes(id);
      const isSynchronisedSuccess = semantics.synchronise === true && childStatus === "success"
        && (!semantics.selected_child_ids.length || !childWasSelected);
      if (isSynchronisedSuccess) {
        return {
          state: "skipped",
          reason: "Skipped because synchronised parallel preserves this child's recorded success.",
        };
      }
      return { state: "not-recorded", reason: "Parallel parent was visited, but this child was not recorded on the tick." };
    }
    if (visits.previous.ids.has(id)) {
      return { state: "not-recorded", reason: "It was visited on the preceding tick but not recorded on this tick." };
    }
    return { state: "not-recorded", reason: "It was not recorded on this tick and no control-flow reason can be proven." };
  }

  /** Current runtime activation: ticked, resumed, skipped, blocked, or not-recorded. */
  function activationState(graph, nodeIdValue, runtime) {
    return activationEvidence(graph, nodeIdValue, runtime).state;
  }

  /** A non-speculative, operator-facing explanation for ``activationState``. */
  function activationReason(graph, nodeIdValue, runtime) {
    return activationEvidence(graph, nodeIdValue, runtime).reason;
  }

  /** Describe the hidden descendants when a rendered tree row is collapsed. */
  function collapsedSubtree(tree, requestedId) {
    const graph = normalizedTree(tree);
    const rootId = resolveTreeId(graph, requestedId);
    if (!rootId) return null;
    const descendantIds = subtreeIds(graph, rootId, { includeRoot: false });
    return {
      rootId,
      descendantIds,
      count: descendantIds.length,
      reason: "collapsed",
    };
  }

  function searchableNodeText(node) {
    return [node.id, node.name, node.label, node.type, node.node_type, node.node_class, node.feedback, node.message]
      .map(normalizedText).join(" ");
  }

  /**
   * Derive a compact render DTO from a normalised or legacy tree.  `expandedIds`
   * controls which visible composites expose descendants; omitted means expand
   * all. `mode: "collapsed"` starts with roots closed, while `full` and
   * `semantic` retain the complete hierarchy unless explicitly collapsed.
   */
  function semanticSkeleton(tree, options) {
    const graph = normalizedTree(tree);
    const settings = options && typeof options === "object" ? options : {};
    const mode = ["full", "semantic", "collapsed"].includes(settings.mode) ? settings.mode : "semantic";
    const requestedExpanded = usableIdSet(settings.expandedIds ?? settings.expanded);
    const hasExpanded = settings.expandedIds != null || settings.expanded != null;
    const focusId = resolveTreeId(graph, settings.focusId ?? settings.activeId ?? settings.nodeId);
    const search = normalizedText(settings.search ?? settings.q);
    const maxNodes = Number.isFinite(Number(settings.maxNodes)) ? Math.max(0, Math.floor(Number(settings.maxNodes))) : Infinity;
    const matches = new Set(search ? graph.nodes.filter(node => searchableNodeText(node).includes(search)).map(node => node.id) : []);
    const forcedExpanded = new Set();
    const forcePath = id => ancestorIds(graph, id).forEach(ancestor => forcedExpanded.add(ancestor));
    if (focusId) {
      pathToNode(graph, focusId).forEach(id => forcedExpanded.add(id));
      ancestorIds(graph, focusId).forEach(id => forcedExpanded.add(id));
    }
    matches.forEach(forcePath);
    const runtime = settings.runtime;
    const visits = runtimeVisitations(runtime);
    const nodes = [];
    const edges = [];
    const collapsedSubtrees = [];
    const seen = new Set();
    let truncated = false;
    const shouldExpand = id => {
      if (forcedExpanded.has(id)) return true;
      if (hasExpanded) return requestedExpanded.has(id);
      return mode !== "collapsed";
    };
    const visit = (id, depth, parentId) => {
      if (!id || seen.has(id)) return;
      if (nodes.length >= maxNodes) {
        truncated = true;
        return;
      }
      seen.add(id);
      const node = graph.byId[id];
      const semantics = nodeSemantics(node);
      const children = graph.childrenById[id] || [];
      const expand = shouldExpand(id);
      const collapsedSubtreeDto = children.length && !expand ? collapsedSubtree(graph, id) : null;
      const descendantIds = collapsedSubtreeDto ? collapsedSubtreeDto.descendantIds : [];
      const evidence = activationEvidence(graph, id, runtime, visits);
      const activation = evidence.state;
      nodes.push({
        id,
        parentId: parentId || null,
        depth,
        name: text(node.name || node.label || node.type || node.node_type || id),
        node_class: semantics.node_class,
        nodeClass: semantics.nodeClass,
        kind: semantics.kind,
        category: semantics.category,
        raw_node_class: semantics.raw_node_class,
        rawNodeClass: semantics.rawNodeClass,
        memory: semantics.memory,
        control_flow: semantics.control_flow,
        controlFlow: semantics.controlFlow,
        success_policy: semantics.success_policy,
        successPolicy: semantics.successPolicy,
        synchronise: semantics.synchronise,
        selected_child_ids: semantics.selected_child_ids.slice(),
        selectedChildIds: semantics.selectedChildIds.slice(),
        counters: semantics.counters == null ? null : copiedModelValue(semantics.counters),
        status: text(runtimeStatus(graph, id, visits) || node.status || ""),
        activation,
        reason: evidence.reason,
        focused: id === focusId,
        matchesSearch: matches.has(id),
        childIds: children.slice(),
        collapsed: descendantIds.length > 0,
        collapsedCount: descendantIds.length,
      });
      if (descendantIds.length) {
        collapsedSubtrees.push(collapsedSubtreeDto);
        // They remain structurally reachable, so do not re-add them as
        // disconnected roots in the defensive fallback below.
        descendantIds.forEach(descendantId => seen.add(descendantId));
        return;
      }
      children.forEach(childId => {
        visit(childId, depth + 1, id);
        if (seen.has(childId)) {
          edges.push({
            parentId: id,
            childId,
            label: edgeActivationLabel(node, childId, graph),
            activation: activationEvidence(graph, childId, runtime, visits).state,
          });
        }
      });
    };
    graph.rootIds.forEach(rootId => visit(rootId, 0, null));
    graph.nodes.forEach(node => {
      if (!seen.has(node.id) && !truncated) visit(node.id, 0, null);
    });
    return {
      mode,
      rootIds: graph.rootIds.slice(),
      nodes,
      edges,
      collapsedSubtrees,
      focusId: focusId || null,
      search,
      truncated,
      warnings: graph.warnings.slice(),
    };
  }

  function queryPairs(input) {
    let search = "";
    if (typeof input === "string") {
      const question = input.indexOf("?");
      search = question >= 0 ? input.slice(question + 1) : input;
    } else if (input && typeof input === "object") {
      search = text(input.search || "");
    }
    const hash = search.indexOf("#");
    if (hash >= 0) search = search.slice(0, hash);
    if (search.startsWith("?")) search = search.slice(1);
    if (typeof URLSearchParams !== "undefined") return new URLSearchParams(search);
    return {
      getAll(key) {
        return search.split("&").filter(Boolean).map(pair => pair.split("=")).filter(pair => decodeURIComponent(pair[0].replace(/\+/g, " ")) === key)
          .map(pair => decodeURIComponent((pair.slice(1).join("=") || "").replace(/\+/g, " ")));
      },
    };
  }

  function firstQuery(pairs, names) {
    for (const name of names) {
      const value = pairs.getAll(name).find(item => item !== "");
      if (value != null) return value;
    }
    return null;
  }

  function queryValues(pairs, names) {
    return Array.from(new Set(names.flatMap(name => pairs.getAll(name)).map(item => text(item).trim()).filter(Boolean)));
  }

  function queryBoolean(value) {
    return ["1", "true", "yes", "on"].includes(normalizedText(value));
  }

  /** Parse the debugger's URL state, accepting legacy long-form key aliases. */
  function parseQuery(input) {
    const pairs = queryPairs(input);
    const atRaw = firstQuery(pairs, ["at", "sequence"]);
    const atNumber = atRaw == null ? null : Number(atRaw);
    return {
      trajectoryId: firstQuery(pairs, ["trajectory", "trajectory_id", "run", "run_id"]),
      at: Number.isInteger(atNumber) && atNumber >= 0 ? atNumber : null,
      view: firstQuery(pairs, ["view"]) || DEFAULT_QUERY_STATE.view,
      taskId: firstQuery(pairs, ["task", "task_id"]),
      search: firstQuery(pairs, ["q", "search", "filter"]) || "",
      statuses: queryValues(pairs, ["status"]),
      categories: queryValues(pairs, ["category"]),
      raw: queryBoolean(firstQuery(pairs, ["raw"])),
      focusId: firstQuery(pairs, ["focus", "focus_id", "node"]),
      treeRevision: firstQuery(pairs, ["tree", "tree_revision", "revision"]),
      treeMode: firstQuery(pairs, ["tree_mode", "treeMode", "mode"]) || DEFAULT_QUERY_STATE.treeMode,
    };
  }

  function appendQuery(target, key, value) {
    if (value == null || value === "") return;
    const values = Array.isArray(value) || value instanceof Set ? Array.from(value) : [value];
    values.map(item => text(item).trim()).filter(Boolean).forEach(item => target.push([key, item]));
  }

  /** Serialise URL state deterministically; defaults are intentionally omitted. */
  function serializeQuery(state, options) {
    const source = state && typeof state === "object" ? state : {};
    const pairs = [];
    appendQuery(pairs, "trajectory", source.trajectoryId ?? source.trajectory_id ?? source.runId);
    if (Number.isInteger(source.at) && source.at >= 0) appendQuery(pairs, "at", source.at);
    if (source.view && source.view !== DEFAULT_QUERY_STATE.view) appendQuery(pairs, "view", source.view);
    appendQuery(pairs, "task", source.taskId ?? source.task_id);
    appendQuery(pairs, "q", source.search ?? source.q);
    appendQuery(pairs, "status", source.statuses ?? source.status);
    appendQuery(pairs, "category", source.categories ?? source.category);
    if (source.raw === true) appendQuery(pairs, "raw", "1");
    appendQuery(pairs, "focus", source.focusId ?? source.focus_id ?? source.nodeId);
    appendQuery(pairs, "tree", source.treeRevision ?? source.tree_revision ?? source.revision);
    const mode = source.treeMode ?? source.tree_mode;
    if (mode && mode !== DEFAULT_QUERY_STATE.treeMode) appendQuery(pairs, "tree_mode", mode);
    const query = pairs.map(([key, value]) => `${encodeURIComponent(key)}=${encodeURIComponent(value)}`).join("&");
    return options && options.prefix === false ? query : (query ? `?${query}` : "");
  }

  return {
    COLLAPSIBLE_EVENT_TYPES,
    DEFAULT_QUERY_STATE,
    statusCategory,
    statusClass,
    eventType,
    eventPayload,
    eventCategory,
    eventStatus,
    eventTimeMs,
    isCollapsibleEvent,
    shortTrajectoryId,
    groupEvents,
    eventSearchText,
    matchesEvent,
    filterEvents,
    supervisorCheckpoints,
    normalizeTree,
    nodeSemantics,
    ancestorIds,
    pathToNode,
    subtreeIds,
    treeVisibility,
    visibleTreeIds,
    edgeActivationLabel,
    activationState,
    activationReason,
    collapsedSubtree,
    semanticSkeleton,
    parseQuery,
    serializeQuery,
    // Descriptive aliases keep browser call sites readable.
    treeAncestorIds: ancestorIds,
    treePathIds: pathToNode,
    treeSubtreeIds: subtreeIds,
    inferNodeSemantics: nodeSemantics,
    deriveSemanticSkeleton: semanticSkeleton,
    parseQueryState: parseQuery,
    serializeQueryState: serializeQuery,
  };
});
