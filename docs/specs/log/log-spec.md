# `dev.cajeta.robotica.log` — Specification

The telemetry & observability **infra** layer of cajeta-robotica. It records and streams
robot state through two industry wire formats — the **MCAP** self-describing log container
and the **Foxglove WebSocket protocol v1** — so the whole Cajeta stack is introspectable and
plugs into the existing observability ecosystem (Foxglove Studio, the MCAP tooling) without
adopting ROS/DDS. See the family spec [`../robotica-spec.md`](../robotica-spec.md) §2.m and
the research
[`../../research/robotics-stack-part2-perception-to-ai.md`](../../research/robotics-stack-part2-perception-to-ai.md) §E.1.

These are **interop wire formats, not math** (research §E.1): the deliverable is a correct
byte-for-byte codec/server, fully testable against recorded byte streams with no hardware.

---

## 1. Definition

`log` provides a **reader and writer for the MCAP container** (8 magic bytes start/end,
ID-keyed Schema/Channel/Message records, optional indexed/chunked/compressed body) and a
**Foxglove WebSocket v1 server** (subprotocol `foxglove.websocket.v1`; JSON `op`-field control
frames + 1-byte-opcode binary frames carrying `json`/`protobuf`/`ros1`/`cdr` encodings). It is
a cross-cutting infra package: it transports **opaque serialized payloads** produced elsewhere;
it neither defines message schemas nor performs robot math.

### 1.a Capability summary
- 1.a.i — MCAP **writer**: emit a well-formed, optionally chunked/indexed/compressed log of
  Schema/Channel/Message records with start/end magic and a summary section.
- 1.a.ii — MCAP **reader**: linear (streaming) and indexed (seek/filter) decode of any
  conformant MCAP file, with magic/opcode/CRC validation.
- 1.a.iii — Foxglove WebSocket **server**: complete the subprotocol handshake, run the JSON
  control plane (server-info, advertise/unadvertise, subscribe/unsubscribe, status).
- 1.a.iv — Foxglove **binary streaming**: push live channel messages as 1-byte-opcode binary
  frames in the negotiated encoding to subscribed clients.
- 1.a.v — A small **bridge / encoding registry** tying the two: replay an MCAP file to a
  Foxglove client, or tee a live channel into both an MCAP writer and Foxglove subscribers.

### 1.b Conventions (NORMATIVE for this package)
> This package **inherits** the family's normative conventions from `transform` §1.b **by
> reference** — quaternion storage, twist/wrench ordering, active-transform composition,
> radians/meters/SI-seconds units, and the `Time`/`Duration` types. It does **not** redefine
> them. `log` transports payloads; their geometric meaning is whatever `transform` fixed.
> The conventions below are **additional, wire-format** decisions local to `log`.
- 1.b.i — **MCAP magic** is the 8-byte sequence `0x89 4D 43 41 50 30 0D 0A` (`\x89MCAP0\r\n`),
  written verbatim at the **start and end** of every file (research §E.1).
- 1.b.ii — **MCAP record framing:** every record is `opcode (uint8) · length (uint64) ·
  content (length bytes)`. All multi-byte integers are **little-endian**. Strings are
  `length (uint32) · UTF-8 bytes`; byte blobs are `length (uint32) · bytes`; maps/arrays are
  `byte-length (uint32) · packed entries`.
- 1.b.iii — **MCAP opcodes** (the ID-keyed record kinds, research §E.1): `0x01` Header,
  `0x02` Footer, `0x03` Schema, `0x04` Channel, `0x05` Message, `0x06` Chunk,
  `0x07` MessageIndex, `0x08` ChunkIndex, `0x09` Attachment, `0x0A` AttachmentIndex,
  `0x0B` Statistics, `0x0C` Metadata, `0x0D` MetadataIndex, `0x0E` SummaryOffset,
  `0x0F` DataEnd. IDs (`schema_id`, `channel_id`) are the cross-record keys.
- 1.b.iv — **MCAP timestamps** are `uint64` **nanoseconds** (`log_time`, `publish_time`,
  chunk `message_start_time`/`message_end_time`), bridged to/from `transform`'s `Time`.
- 1.b.v — **Foxglove subprotocol** is the literal WebSocket subprotocol string
  `foxglove.websocket.v1`. JSON control frames carry a string `op` field; binary frames carry
  a **1-byte opcode** prefix; message `encoding` is one of `json` / `protobuf` / `ros1` /
  `cdr` (research §E.1).
- 1.b.vi — **CRC-32 (IEEE 802.3)** is the checksum used for MCAP chunk and footer integrity;
  `0` denotes "not computed". **[scope: secondary source — confirm polynomial/seed against the
  MCAP conformance vectors before the plan.]**

### 1.c Non-goals
- 1.c.i — **No schema authoring or payload (de)serialization.** Message bodies are opaque
  bytes; protobuf/ros1/cdr/json *encoding* of robot types is the producer's job (and out of
  family scope). `log` carries and indexes them, never interprets them.
- 1.c.ii — **No robot math** — no transforms, fusion, or estimation (those are the math
  packages; `log` depends *down* on `transform` only for the `Time` type).
- 1.c.iii — **No visualization UI** — `log` is the server/codec side; Foxglove Studio is the
  unmodified third-party client.
- 1.c.iv — **No new transport stack** — TCP/WebSocket framing rides Cajeta's `net` substrate
  (see §3.f open question); `log` owns the *application* protocol above the socket.
- 1.c.v — **No compression algorithm implementation in-tree** beyond a pluggable codec
  interface; `none` is always supported, `lz4`/`zstd` are codec plug-ins (§1.b.vi / §3.f).

---

## 2. Features

### 2.a MCAP record model & primitive codec
Definition: the value types for the fifteen MCAP records (§1.b.iii) and the little-endian,
length-prefixed primitive (de)serializers (§1.b.ii) every reader/writer path shares.
- 2.a.i (use case) — Encode/decode the primitive grammar: `uint8/16/32/64` LE, length-prefixed
  UTF-8 string, length-prefixed byte blob, and the `map<string,string>` / array forms.
- 2.a.ii — Frame and unframe a record: `opcode · uint64 length · content`, including rejecting a
  truncated or over-long length against the remaining buffer.
- 2.a.iii — Model each record as a typed value: Header `(profile, library)`; Footer
  `(summary_start, summary_offset_start, summary_crc)`; Schema `(id, name, encoding, data)`;
  Channel `(id, schema_id, topic, message_encoding, metadata)`; Message
  `(channel_id, sequence, log_time, publish_time, data)`.
- 2.a.iv — Model the index/summary records: Chunk, MessageIndex, ChunkIndex, Statistics,
  Attachment(+Index), Metadata(+Index), SummaryOffset, DataEnd.
- 2.a.v — Compute and verify CRC-32 (§1.b.vi) over a record/chunk byte range.
- Acceptance: round-trip — `decode(encode(record)) == record` for every record kind, and the
  primitive codecs match recorded golden bytes.

### 2.b MCAP writer
Definition: assemble records (§2.a) into a conformant file: magic · Header · data section ·
DataEnd · summary section · summary-offset section · Footer · magic.
- 2.b.i — Open a writer with `profile` + `library`, registering Schemas and Channels (assigning
  stable `schema_id`/`channel_id`) before their first Message.
- 2.b.ii — Append a Message on a channel (`sequence`, `log_time`, `publish_time`, opaque data),
  in either **unchunked** or **chunked** mode.
- 2.b.iii — In chunked mode: accumulate records into a Chunk (bounded by size/time), optionally
  compress via the pluggable codec (`none`/`lz4`/`zstd`, §1.c.v), and emit per-chunk
  MessageIndex records.
- 2.b.iv — On close: write the summary section (repeated Schema/Channel, Statistics, ChunkIndex,
  Attachment/Metadata indexes), the SummaryOffset section, the Footer (with `summary_crc`), and
  the trailing magic.
- 2.b.v — Attach side data: Attachment records (named blobs) and Metadata records (key-value
  maps), reflected in the summary indexes.
- Acceptance: output validates against the MCAP conformance reader/test data; a writer→reader
  round-trip reproduces every Schema/Channel/Message exactly.

### 2.c MCAP reader (streaming + indexed)
Definition: decode any conformant MCAP — a forward streaming path and a seek/filter path that
uses the summary indexes.
- 2.c.i — **Streaming read:** validate the leading magic, then iterate records in file order,
  transparently descending into (and decompressing) Chunks, surfacing Schema/Channel/Message in
  log order.
- 2.c.ii — **Indexed read:** parse the Footer → summary section → ChunkIndex/MessageIndex, and
  seek to messages by channel/topic and `[start_time, end_time]` without scanning the body.
- 2.c.iii — Resolve Channel→Schema by ID and expose the metadata maps, statistics, and the
  valid time range of the file.
- 2.c.iv — **Validate & recover:** verify start/end magic and chunk/footer CRCs; on a truncated
  or corrupt file, recover the records preceding the damage (a partially-written robot log).
- 2.c.v — Enumerate Attachments and Metadata by name.
- Acceptance: reads the official MCAP conformance corpus (including chunked/compressed/indexed
  variants) byte-accurately; rejects bad magic/opcode/CRC deterministically.

### 2.d Foxglove WebSocket server — handshake & JSON control plane
Definition: a server speaking `foxglove.websocket.v1` (§1.b.v): the subprotocol handshake plus
the JSON `op`-field control messages, atop Cajeta `net` (§1.c.iv).
- 2.d.i — Complete the WebSocket upgrade negotiating subprotocol `foxglove.websocket.v1`;
  reject clients that do not offer it.
- 2.d.ii — On connect, send `serverInfo` (name, `capabilities`, `supportedEncodings`,
  `metadata`) and emit `status` (level/message) frames for operational events.
- 2.d.iii — Handle inbound client control frames: `subscribe`/`unsubscribe` (mapping a client
  `subscriptionId` → server `channelId`), and client-advertise/unadvertise if client-publish is
  enabled.
- 2.d.iv — Send `advertise`/`unadvertise` as the set of available channels changes (each
  advertised channel: `id`, `topic`, `encoding`, `schemaName`, `schema`, `schemaEncoding`).
- 2.d.v — Multi-client session state: per-connection subscription tables, backpressure/slow-
  consumer policy, and clean teardown on disconnect.
- 2.d.vi — **[scope]** Optional capabilities behind `serverInfo.capabilities` — `parameters`
  (get/set/subscribe), `services` (request/response), `assets` (fetch), `connectionGraph` —
  declared but deferrable; gate each on its capability flag.
- Acceptance: handshake + control-frame exchange reproduces a recorded Foxglove-Studio session
  capture (JSON frames match field-for-field).

### 2.e Foxglove WebSocket server — binary message streaming
Definition: deliver live channel data to subscribers as 1-byte-opcode binary frames (§1.b.v).
- 2.e.i — Publish a message on a channel: frame it as binary opcode `0x01` MessageData
  (`subscriptionId` uint32, `receiveTimestamp` uint64 ns, payload bytes) and fan it out only to
  clients subscribed to that channel.
- 2.e.ii — Emit the binary `Time` frame (opcode `0x02`) so the client clock can follow the
  robot/log clock.
- 2.e.iii — Carry the payload verbatim in the channel's negotiated `encoding`
  (`json`/`protobuf`/`ros1`/`cdr`); `log` does not transcode between encodings.
- 2.e.iv — **[scope]** Decode inbound client binary frames (client MessageData / service-call
  request) when the corresponding capability is enabled.
- Acceptance: a subscribe → stream sequence reproduces a recorded binary-frame capture
  (opcode + header + payload bytes match), and unsubscribed channels are never delivered.

### 2.f Bridge & encoding registry
Definition: the thin glue tying §2.b–2.e together so a Cajeta robot is observable with minimal
wiring; a registry maps a logical channel to its `(MCAP message_encoding ↔ Foxglove encoding,
schema)` pair.
- 2.f.i — **Live tee:** publish one logical channel simultaneously into an MCAP writer (§2.b)
  and to Foxglove subscribers (§2.e), assigning consistent IDs/schemas in both.
- 2.f.ii — **Replay:** stream an existing MCAP file (§2.c) to a connected Foxglove client,
  advertising its Channels/Schemas and pacing Messages by `log_time`.
- 2.f.iii — Maintain the encoding registry mapping a channel's serialization name to both the
  MCAP `message_encoding`/Schema and the Foxglove `encoding`/`schemaEncoding`, so the same
  opaque payload flows to both sinks without transcoding.
- 2.f.iv — Bridge `transform.Time` ↔ MCAP `uint64` ns (§1.b.iv) ↔ Foxglove `Time` frame at the
  one place timestamps cross the boundary.
- Acceptance: tee then read-back yields an MCAP whose messages equal what a recording Foxglove
  client received; replay of a golden MCAP reproduces a golden Foxglove session.

---

## 3. Acceptance criteria (beyond per-feature tests)
- 3.a — **Codec conformance against recorded bytes.** Per the driver-layer family pillar
  (family §3.b), every wire path is tested against **recorded byte streams** — the official
  MCAP conformance corpus and captured Foxglove-Studio frame logs — not live hardware.
- 3.b — **Byte-exact round-trips.** `write → read` reproduces all records; `encode → decode`
  is identity for every record/frame kind; CRCs validate.
- 3.c — **Robust rejection.** Bad magic, unknown/oversized opcodes/lengths, and CRC mismatches
  are rejected deterministically; truncated logs recover the intact prefix (§2.c.iv).
- 3.d — **Hardware-free & board-agnostic.** The whole suite runs in CI on x86-64 and at least
  one RISC-V target with no devices and no non-portable native lib in the core (family §3.c);
  any compression codec is an optional, isolated plug-in (§1.c.v, §3.f).
- 3.e — **No upward dependencies.** `log` imports only `transform` (for `Time`) and Cajeta
  stdlib (`net`, bytes) — nothing else in `dev.cajeta.robotica.*`, and never a math/driver
  package (family §1.b.ii).
- 3.f — **Convention inheritance.** A test asserts `log` reuses `transform`'s `Time`/units types
  unchanged at the timestamp boundary (no redefinition).

## 4. Deliverables
- 4.a — `dev.cajeta.robotica.log` implementing §2, with the §3 test suite (MCAP conformance
  corpus + recorded Foxglove captures committed as fixtures).
- 4.b — A short wire-conventions reference (§1.b: MCAP magic/opcodes/framing, Foxglove
  subprotocol/opcodes/encodings) for downstream producers.
- 4.c — A plan at `agents/log-plan.md` (authored when this spec is approved), TDD-ordered:
  primitive codec & record model (§2.a) → MCAP writer (§2.b) → MCAP reader (§2.c) → Foxglove
  handshake/control plane (§2.d) → Foxglove binary streaming (§2.e) → bridge/registry (§2.f).

## 5. References (research-grounded)
- **MCAP** self-describing log container — 8 magic bytes start/end, ID-keyed
  Schema/Channel/Message records, any serialization — research §E.1; source `mcap.dev/spec`.
  **[verified]**
- **Foxglove WebSocket protocol v1** — subprotocol `foxglove.websocket.v1`, JSON `op`-field
  control frames + 1-byte-opcode binary frames, encodings `json`/`protobuf`/`ros1`/`cdr` —
  research §E.1; source `github.com/foxglove/ws-protocol/blob/main/docs/spec.md`. **[verified]**
- MCAP record **opcode table, CRC-32 polynomial/seed, and chunk/summary layout** beyond the
  §E.1 summary — to be confirmed against `mcap.dev/spec` and the conformance vectors when the
  plan is written. **[scope: detail beyond the verified research summary]**
- Compression codecs **lz4 / zstd** for chunk compression — pluggable, not part of the verified
  surface; `none` is the conformance baseline. **[scope]**
