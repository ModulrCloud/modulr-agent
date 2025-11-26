#!/usr/bin/env python3
"""
Debug script to monitor rosbridge WebSocket connections and subscriptions
Run this while the Rust agent is running to see what's happening
"""

import asyncio
import websockets
import json
import sys

async def monitor_rosbridge():
    # First, just connect and see what's happening
    uri = "ws://localhost:9090"
    print(f"Connecting to rosbridge at {uri}...")
    
    try:
        async with websockets.connect(uri) as websocket:
            print("✅ Connected to rosbridge!")
            print("\nMonitoring rosbridge messages...")
            print("(Run the Rust agent in another terminal to see subscription messages)\n")
            
            # Send a test subscribe to see the format
            subscribe_msg = {
                "op": "subscribe",
                "topic": "/camera/image_raw",
                "type": "sensor_msgs/msg/Image"
            }
            print(f"Sending test subscription: {json.dumps(subscribe_msg, indent=2)}")
            await websocket.send(json.dumps(subscribe_msg))
            print("✅ Test subscription sent!\n")
            
            # Monitor incoming messages
            message_count = 0
            try:
                async for message in websocket:
                    try:
                        msg = json.loads(message)
                        message_count += 1
                        
                        # Show first few messages to understand format
                        if message_count <= 3:
                            print(f"📨 Message #{message_count}:")
                            print(f"   op: {msg.get('op', 'N/A')}")
                            if 'topic' in msg:
                                print(f"   topic: {msg['topic']}")
                            if 'type' in msg:
                                print(f"   type: {msg['type']}")
                            print()
                        elif message_count % 50 == 0:
                            print(f"   (Received {message_count} messages so far...)")
                    except json.JSONDecodeError:
                        # Might be binary data
                        print(f"📦 Received binary message ({len(message)} bytes)")
                        
            except KeyboardInterrupt:
                print(f"\n\n📊 Total messages received: {message_count}")
                print("Monitoring stopped.")
                    
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    print("=" * 60)
    print("Rosbridge Debug Monitor")
    print("=" * 60)
    print("\nThis will show what messages rosbridge is sending/receiving.")
    print("Start the Rust agent in another terminal to see its subscription.\n")
    asyncio.run(monitor_rosbridge())

