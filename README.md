# v5-copro

v5-copro is a PROS template which simplifies serial communications between a coprocessor and the VEX V5 Brain.

## The Protocol

### Limitations

- 921600 baud only
- no daisy chaining
- half-duplex only
- synchronous only

### Packet Structure

#### Leader To Follower Packet

```
[8-bit ID] [CRC16] [payload] [0x0]
```

#### Response Packet

```
[CRC16] [payload] [0x0]
```
