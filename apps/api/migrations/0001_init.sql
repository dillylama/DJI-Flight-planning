-- M400/L3 sync backend: initial schema. Timestamps are ms since epoch; ids are UUIDs.

CREATE TABLE projects (
  id         TEXT PRIMARY KEY,
  name       TEXT NOT NULL,
  created_at INTEGER NOT NULL,
  updated_at INTEGER NOT NULL
);

-- One row per published mission package. Immutable once inserted.
CREATE TABLE versions (
  id             TEXT PRIMARY KEY,
  project_id     TEXT NOT NULL REFERENCES projects(id),
  n              INTEGER NOT NULL,          -- 1, 2, 3 ... per project
  created_at     INTEGER NOT NULL,
  note           TEXT,
  manifest       TEXT NOT NULL,             -- JSON summary of mission.json
  mission_key    TEXT NOT NULL,             -- R2 key
  mission_size   INTEGER NOT NULL,
  mission_sha256 TEXT NOT NULL,
  dem_key        TEXT,
  dem_size       INTEGER,
  dem_sha256     TEXT
);
CREATE UNIQUE INDEX versions_project_n ON versions(project_id, n);

CREATE TABLE devices (
  id         TEXT PRIMARY KEY,
  name       TEXT NOT NULL,
  token_hash TEXT NOT NULL,                 -- SHA-256 hex of the bearer token
  created_at INTEGER NOT NULL,
  last_seen  INTEGER,
  revoked    INTEGER NOT NULL DEFAULT 0
);
CREATE UNIQUE INDEX devices_token_hash ON devices(token_hash);

CREATE TABLE pairing_codes (
  code       TEXT PRIMARY KEY,
  created_at INTEGER NOT NULL,
  expires_at INTEGER NOT NULL,
  used       INTEGER NOT NULL DEFAULT 0
);

CREATE TABLE logs (
  id         TEXT PRIMARY KEY,
  device_id  TEXT NOT NULL,
  project_id TEXT,
  version_id TEXT,
  created_at INTEGER NOT NULL,
  r2_key     TEXT NOT NULL,
  size       INTEGER NOT NULL,
  summary    TEXT                           -- JSON
);
CREATE INDEX logs_device ON logs(device_id, created_at);
CREATE INDEX logs_project ON logs(project_id, created_at);
