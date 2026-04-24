# IntentApi — Publishing Normalized Intent Messages

## Introduction

`IntentApi` is the thin ROS2 publishing layer that bridges the CLI and any downstream behavior manager listening on `/intent`. The design choice to use `std_msgs/String` carrying JSON rather than a custom message type is deliberate: the intent vocabulary is still evolving, and a custom `.msg` file would require recompilation on every vocabulary change. JSON strings let us iterate freely until the schema stabilizes.

## Inheritance: Standing on BaseApi's Shoulders

`IntentApi` inherits from `BaseApi`, which itself inherits from ROS2's `Node`. This means every `IntentApi` instance *is* a ROS2 node — it owns a node name in the graph, can publish and subscribe, and participates in DDS discovery.

```python
class IntentApi(base.BaseApi):

    def __init__(self, config_manager: cm.ConfigManager = None):
        super().__init__("intent_api", config_manager)
        self.intent_pub = self.create_publisher(String, "/intent", 10)
```

The publisher is created with a queue depth of 10. Because `IntentApi` is initialized lazily (created on first use), there is a real risk of publishing before DDS has completed subscriber discovery. The caller (`RobotController`) adds a 0.5 s sleep after first construction to absorb this latency.

## The Publish Method

The single public method serializes the intent to JSON and hands it to the ROS2 publisher:

```python
def publish(self, name: str, source: str, slots: dict) -> None:
    msg = String()
    msg.data = json.dumps({"name": name, "source": source, "slots": slots})
    self.intent_pub.publish(msg)
    self.log_info(f"Intent published: {msg.data}")
```

The wire format is fixed:
```json
{"name": "count_objects", "source": "cli", "slots": {"object_type": "chair"}}
```

- **name**: the intent identifier (e.g. `stop`, `explore`)
- **source**: who originated the intent (`"cli"` always for this module)
- **slots**: key-value parameters; empty dict `{}` for zero-argument intents

## Data Flow

```
CLI REPL
   │  user types "intent count_objects object_type=chair"
   ▼
CommandDispatcher.execute("intent.count_objects", {"object_type": "chair"})
   │
   ▼
RobotController.publish_intent_count_objects(object_type="chair")
   │
   ▼
RobotController.publish_intent("count_objects", {"object_type": "chair"})
   │
   ▼
IntentApi.publish("count_objects", "cli", {"object_type": "chair"})
   │
   ▼
/intent  std_msgs/String  JSON
```

## Possible Improvements

- **QoS — transient local**: switching to `QoSProfile(durability=TRANSIENT_LOCAL)` would let late-joining subscribers receive the last message, eliminating the 0.5 s sleep workaround entirely.
- **Custom message type**: once the vocabulary stabilizes, migrating to a typed `Intent.msg` with fields `name`, `source`, and a serialized slots string would provide compile-time checking and introspection.
- **Async publishing**: for high-frequency intent streams, moving to an async executor would prevent publish calls from blocking the REPL thread.
