import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import asyncio
import time
from bleak import BleakClient

class CmdVelSubscriber(Node):
    def __init__(self):
        super().__init__('cmdvel_sub')
        self.cmd_vel_topic = '/cmd_vel'
        self.sub = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmdCB,
            20
        )
        self.get_logger().info(f"Subscribed to {self.cmd_vel_topic}")

    def cmdCB(self, msg: Twist):
        self.get_logger().info(
            f"Received cmd_vel -> linear.x: {msg.linear.x:.2f}, angular.z: {msg.angular.z:.2f}"
        )



class CarBLE:
    def __init__(self, address="EC:B1:B6:05:9D:50"):
        self.address = address
        self.client = None
        self.response_data = None
        self.response_received = asyncio.Event()
    
    async def connect(self):
        """Connect to the BLE device and discover services"""
        try:
            self.client = BleakClient(self.address)
            await self.client.connect()
            
            if self.client.is_connected:
                print("Connected to BLE device")
                
                # Services are automatically discovered when connecting with BleakClient
                # No need to call get_services() separately in recent versions
                print("Services discovered automatically")
                
                # Set up notification handler
                await self.setup_notifications()
                
                return True
            else:
                print("Connection failed")
                return False
                
        except Exception as e:
            print(f"Connection error: {e}")
            return False
    
    async def setup_notifications(self):
        """Setup notification handler for response characteristic"""
        # Try common notification characteristic UUIDs

        
        def notification_handler(sender, data):
            """Handle incoming notifications"""
            try:
                response = data.decode('utf-8')
                print(f"Received notification from {sender}: {response}")
                self.response_data = response
                self.response_received.set()  # Signal that we received data
            except Exception as e:
                print(f"Error decoding notification: {e}")
        
        # Get services from the client
        # CORRECTED: Use client.services instead of get_services()
        # ensure services are populated and subscribe to ANY notify/indicate chars
        print("Listing services and subscribing to notifies (if present):")
        for service in self.client.services:
            print(f"Service: {service.uuid}")
            for char in service.characteristics:
                print(f"  Char: {char.uuid}  props={char.properties}")
                if 'notify' in char.properties or 'indicate' in char.properties:
                    try:
                        await self.client.start_notify(char.uuid, notification_handler)
                        print(f"  -> start_notify OK on {char.uuid}")
                    except Exception as e:
                        print(f"  -> start_notify failed on {char.uuid}: {e}")


    
    async def send_at_command(self, command, response_timeout=3.0):
        """
        Send an AT command to the device and wait for response
        """
        if not self.client or not self.client.is_connected:
            print("Not connected")
            return None
        
        try:
            # Reset response data and event
            self.response_data = None
            self.response_received.clear()
            
            # Prepare command (add carriage return if needed)
            if not command.endswith('\r\n'):
                command += '\r\n'
            command_bytes = command.encode('utf-8')
            
            print(f"Sending AT command: {command_bytes}")
            

            # CORRECTED: Use client.services instead of get_services()
            for service in self.client.services:
                for char in service.characteristics:
                    if "write" in char.properties:
                        print(f"Writing to characteristic: {char.uuid}")
                        try:
                            await self.client.write_gatt_char(char.uuid, command_bytes)
                            break  # Successfully wrote to a characteristic
                        except Exception as e:
                            print(f"Failed to write to {char.uuid}: {e}")
            
            # Wait for response with timeout
            try:
                await asyncio.wait_for(self.response_received.wait(), timeout=response_timeout)
                return self.response_data
            except asyncio.TimeoutError:
                print("Response timeout")
                # Try to read directly from characteristics
                return await self.try_direct_read()
            
        except Exception as e:
            print(f"Error sending AT command: {e}")
            return None
    

    
    async def disconnect(self):
        """Disconnect from the device"""
        if self.client:
            await self.client.disconnect()
            print("Disconnected")

async def ble_main():
    car = CarBLE("EC:B1:B6:05:9D:50")
    
    connected = await car.connect()
    if connected:
        try:
            # Example AT commands to try
            at_commands = [
                "AT",           # Basic attention command
                "AT+VER?",       # Version request
                "AT+NAME?",      # Device name
            ]
            
            # Try each command
            for cmd in at_commands:
                print(f"\n--- Sending: {cmd} ---")
                response = await car.send_at_command(cmd)
                if response:
                    print(f"Response: {response}")
                else:
                    print("No response received")
                
                # Short delay between commands
                await asyncio.sleep(1)
            
        except Exception as e:
            print(f"Error during communication: {e}")
        
        finally:
            await car.disconnect()




def main(args=None):
    rclpy.init(args=args)
    node = CmdVelSubscriber()
    # Run the BLE demo
    asyncio.run(ble_main())
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
