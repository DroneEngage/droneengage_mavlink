# DroneEngage Message Routing

## Overview

DroneEngage uses a sophisticated message routing system that supports both local inter-module communication and remote inter-unit communication through a central server. The system handles three main message types: system messages, JSON messages, and binary messages.

## Message Types

### 1. System Messages (`sendSYSMSG`)
- **Purpose**: System-level communication sent directly to the server
- **Target**: Always `ANDRUAV_PROTOCOL_SENDER_COMM_SERVER`
- **Routing Type**: `CMD_COMM_SYSTEM`
- **Flow**: Module → Server
- **Use Case**: System administration, configuration, authentication

### 2. JSON Messages (`sendJMSG`)
- **Purpose**: Text-only JSON communication
- **Routing**: Determined by `internal_message` parameter
- **Structure**: Pure JSON string
- **Use Case**: Commands, status updates, configuration data

### 3. Binary Messages (`sendBMSG`)
- **Purpose**: Messages with binary payload
- **Structure**: `[JSON_HEADER][NULL_BYTE][BINARY_PAYLOAD]`
- **Routing**: Same as JSON messages (determined by `internal_message`)
- **Use Case**: MAVLink packets, images, sensor data, file transfers

## Message Routing Logic

### `internal_message` Parameter

The `internal_message` boolean parameter controls the message scope:

#### `internal_message = true` (Local Communication)
- **Routing Type**: `CMD_TYPE_INTERMODULE`
- **Behavior**:
  - Message stays within the local unit
  - `de_comm` module receives the message
  - `de_comm` forwards to other subscribed modules on the same unit
  - **NEVER forwarded to the server**
- **Use Case**: Internal module coordination, local data sharing

#### `internal_message = false` (Remote Communication)
- **Routing Types**:
  - `CMD_COMM_GROUP` (if `targetPartyID` is empty)
  - `CMD_COMM_INDIVIDUAL` (if `targetPartyID` is specified)
- **Behavior**:
  - Message sent to server first
  - Server forwards to other units based on routing type
  - Server handles message distribution and filtering
- **Use Case**: Inter-unit communication, remote control, data sharing

## Routing Decision Tree

```
internal_message = true
├── CMD_TYPE_INTERMODULE
├── Local unit only
└── de_comm → subscribed modules

internal_message = false
├── targetPartyID empty → CMD_COMM_GROUP
│   ├── Module → Server → All units
│   └── Broadcast to all units
├── targetPartyID set → CMD_COMM_INDIVIDUAL  
│   ├── Module → Server → Specific unit
│   └── Point-to-point communication
└── Server-mediated routing
```

## Message Structure

### JSON Message Structure
```json
{
  "INTERMODULE_MODULE_KEY": "module_key",
  "ANDRUAV_PROTOCOL_TARGET_ID": "target_party_id",
  "INTERMODULE_ROUTING_TYPE": "routing_type",
  "ANDRUAV_PROTOCOL_MESSAGE_TYPE": message_type_id,
  "ANDRUAV_PROTOCOL_MESSAGE_CMD": { ... }
}
```

### Binary Message Structure
```
[JSON_HEADER][NULL_BYTE][BINARY_PAYLOAD]
```

- **JSON Header**: Same structure as JSON messages
- **NULL Byte**: Single byte with value `0` (`\0`) as delimiter
- **Binary Payload**: Raw binary data (optional, can be empty)

## Binary vs Text Detection

The receiver determines message type by examining the message ending:

```cpp
m_is_binary = !(full_message[full_message_length - 1] == 125 || 
                (full_message[full_message_length - 2] == 125));
```

- **Text Message**: Ends with `}` (ASCII 125)
- **Binary Message**: Ends with binary data (not `}`)

## Key Components

### Module Keys
- **`INTERMODULE_MODULE_KEY`**: Unique identifier for the sending module
- Used for message tracking and routing decisions

### Target IDs
- **`ANDRUAV_PROTOCOL_TARGET_ID`**: Destination identifier
- Empty string = broadcast to all
- Specific string = individual targeting

### Routing Types
- **`CMD_TYPE_INTERMODULE`**: Local inter-module communication
- **`CMD_COMM_GROUP`**: Broadcast to all units
- **`CMD_COMM_INDIVIDUAL`**: Point-to-point communication
- **`CMD_COMM_SYSTEM`**: System messages to server

## Message Flow Examples

### Local Module Communication
```cpp
// Message stays within local unit
sendJMSG("", command_data, MESSAGE_ID, true);
```

### Remote Unit Communication
```cpp
// Message goes through server to all units
sendJMSG("", command_data, MESSAGE_ID, false);

// Message goes through server to specific unit
sendJMSG("target_unit_id", command_data, MESSAGE_ID, false);
```

### Binary Data Transfer
```cpp
// Send binary payload with JSON header
sendBMSG("target_id", binary_data, data_length, MESSAGE_ID, false, json_cmd);
```

## Security and Permissions

- Messages include permission levels for access control
- Server validates message permissions before forwarding
- Internal messages bypass server validation (local trust)

## Performance Considerations

- **Local Messages**: Faster, no network latency
- **Remote Messages**: Server-mediated, network latency applies
- **Binary Messages**: Efficient for large data transfers
- **JSON Messages**: Human-readable, easier debugging

## Error Handling

- Messages with invalid routing types are rejected
- Missing required fields cause message rejection
- Network failures trigger retry mechanisms
- Permission violations result in message blocking

## Best Practices

1. **Use `internal_message = true`** for:
   - Module coordination within same unit
   - High-frequency local data sharing
   - Time-sensitive operations

2. **Use `internal_message = false`** for:
   - Inter-unit communication
   - Remote control and monitoring
   - Data synchronization across units

3. **Use binary messages for**:
   - Large data transfers (images, files)
   - Protocol payloads (MAVLink packets)
   - Binary sensor data

4. **Use JSON messages for**:
   - Commands and configuration
   - Status updates
   - Human-readable data
