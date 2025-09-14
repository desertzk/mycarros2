import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import asyncio
import struct
import threading
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
        
        # Initialize BLE connection (you'll need to implement this)
        self.ble_connected = False
        self.car = CarBLE("EC:B1:B6:05:9D:50")
        self.setup_ble_connection()

    def setup_ble_connection(self):
        # Initialize your BLE connection here
        # This will depend on your specific BLE implementation    
        connected = self.car.connect()
        if connected:
            self.ble_connected = True
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
                    response = self.car.send_at_command(cmd)
                    if response:
                        print(f"Response: {response}")
                    else:
                        print("No response received")
                                        
            except Exception as e:
                print(f"Error during communication: {e}")
            pass

    def send_custom_protocol(self, linear_x, angular_z):
        """
        Convert linear and angular velocities to custom protocol format
        Protocol: 0x5A (header), length, ID, function code, data
        """
        # Scale the values to a suitable range for your car
        # Adjust these scaling factors based on your car's capabilities
        linear_scaled = int(linear_x * 100)  # Scale to range -100 to 100
        angular_scaled = int(angular_z * 100)  # Scale to range -100 to 100
        
        # Create data bytes (adjust based on your protocol needs)
        data = struct.pack('bb', linear_scaled, angular_scaled)
        
        # Build the frame
        frame_length = 4 + len(data)   # header(1)+length(1)+device_id(1)+function(1)+data(len)
        frame_header = 0x5A
        device_id = 0x01
        function_code = 0x00  # Movement control function
        
        # Construct the complete frame
        frame = struct.pack('BBBB', frame_header, frame_length, device_id, function_code) + data
        # Debug: Print the frame
        frame_hex = ' '.join(f'{b:02X}' for b in frame)
        self.get_logger().info(f"Sending frame: {frame_hex}")
        # Send via BLE
        if self.ble_connected:
            self.send_ble_data(frame)
    
    def send_ble_data(self, data):
        # data may be bytes or str
        if isinstance(data, (bytes, bytearray)):
            ok = self.car.write_raw(data)
            print("raw write OK" if ok else "raw write failed")
        else:
            response = self.car.send_at_command(data)
            if response:
                print(f"Response: {response}")
            else:
                print("No response received")
        

    def cmdCB(self, msg: Twist):
        self.get_logger().info(
            f"Linear: x={msg.linear.x:.2f}, y={msg.linear.y:.2f}, z={msg.linear.z:.2f} | "
            f"Angular: x={msg.angular.x:.2f}, y={msg.angular.y:.2f}, z={msg.angular.z:.2f}"
        )
        
        # Convert Twist to your custom protocol and send
        self.send_custom_protocol(msg.linear.x, msg.angular.z)

    # override destroy_node so ROS2 shutdown triggers BLE disconnect
    def destroy_node(self):
        try:
            if getattr(self, "ble_connected", False) and getattr(self, "car", None) is not None:
                self.get_logger().info("Disconnecting BLE client...")
                try:
                    self.car.disconnect()   # sync wrapper
                    self.get_logger().info("BLE disconnected")
                except Exception as e:
                    self.get_logger().error(f"Error while disconnecting BLE: {e}")
                finally:
                    self.ble_connected = False
        except Exception as e:
            # log but don't raise - allow normal destroy flow
            self.get_logger().error(f"Exception in destroy_node cleanup: {e}")
        # call parent destroy_node to finish cleanup
        return super().destroy_node()





class CarBLE:
    def __init__(self, address="EC:B1:B6:05:9D:50"):
        self.address = address
        self.client = None
        self.response_data = None
        self._connected = False
        self.response_received = asyncio.Event()
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self._run_loop, daemon=True)
        self.thread.start()


    def _run_loop(self):
        asyncio.set_event_loop(self.loop)
        try:
            self.loop.run_forever()
        finally:
            # attempt a tidy shutdown of async generators and close the loop
            try:
                self.loop.run_until_complete(self.loop.shutdown_asyncgens())
            except Exception:
                pass
            self.loop.close()

    async def _connect(self):
        try:
            self.client = BleakClient(self.address)
            await self.client.connect()
            
            if self.client.is_connected:
                print("Connected to BLE device")
                await self._setup_notifications()
                self._connected = True
                return True
            return False
        except Exception as e:
            print(f"Connection error: {e}")
            return False

    async def _setup_notifications(self):
        def notification_handler(sender, data):
            try:
                response = data.decode('utf-8')
                print(f"Received notification: {response}")
                self.response_data = response
                self.response_received.set()
            except Exception as e:
                print(f"Error decoding notification: {e}")

        for service in self.client.services:
            for char in service.characteristics:
                if 'notify' in char.properties or 'indicate' in char.properties:
                    try:
                        await self.client.start_notify(char.uuid, notification_handler)
                        print(f"Subscribed to notifications on {char.uuid}")
                    except Exception as e:
                        print(f"Failed to subscribe to {char.uuid}: {e}")

    async def _send_at_command(self, command, response_timeout=3.0):
        if not self.client or not self.client.is_connected:
            print("Not connected")
            return None

        self.response_data = None
        self.response_received.clear()
        
        if not command.endswith('\r\n'):
            command += '\r\n'
        command_bytes = command.encode('utf-8')

        for service in self.client.services:
            for char in service.characteristics:
                if "write" in char.properties:
                    print(command_bytes)
                    try:
                        await self.client.write_gatt_char(char.uuid, command_bytes)
                        break
                    except Exception as e:
                        print(f"Failed to write to {char.uuid}: {e}")

        try:
            await asyncio.wait_for(self.response_received.wait(), timeout=response_timeout)
            return self.response_data
        except asyncio.TimeoutError:
            print("Response timeout")
            return None

    async def _disconnect(self):
        # async disconnect coroutine
        try:
            if self.client is not None and getattr(self.client, "is_connected", False):
                try:
                    await self.client.disconnect()
                    print("BLE: disconnected")
                    self._connected = False
                except Exception as e:
                    print(f"BLE disconnect error: {e}")
            # stop the loop so _run_loop can finish
            self.loop.call_soon_threadsafe(self.loop.stop)
        except Exception as e:
            print(f"_disconnect exception: {e}")

    def connect(self):
        future = asyncio.run_coroutine_threadsafe(self._connect(), self.loop)
        return future.result()

    def disconnect(self, timeout=5.0):
        """
        Synchronous wrapper to disconnect and stop the event loop,
        then join the thread.
        """
        try:
            future = asyncio.run_coroutine_threadsafe(self._disconnect(), self.loop)
            # Wait for the coroutine to run (it schedules loop.stop too).
            try:
                future.result(timeout=timeout)
            except Exception:
                # if disconnect timed out/cancelled, attempt to stop loop anyway
                try:
                    self.loop.call_soon_threadsafe(self.loop.stop)
                except Exception:
                    pass
        finally:
            # join the background thread so process can exit cleanly
            self.thread.join(timeout=timeout)
            # Note: loop is closed in _run_loop finally block

    def send_at_command(self, command):
        future = asyncio.run_coroutine_threadsafe(self._send_at_command(command), self.loop)
        return future.result()
    
    # add to CarBLE
    async def _write_raw(self, raw_bytes):
        if not self.client or not self.client.is_connected:
            print("Not connected")
            return False

        for service in self.client.services:
            for char in service.characteristics:
                if ("write" in char.properties) or ("write_without_response" in char.properties):
                    try:
                        await self.client.write_gatt_char(char.uuid, raw_bytes)
                        return True
                    except Exception as e:
                        print(f"raw write failed on {char.uuid}: {e}")
        return False

    # wrapper callable from other thread
    def write_raw(self, raw_bytes):
        future = asyncio.run_coroutine_threadsafe(self._write_raw(raw_bytes), self.loop)
        return future.result()
    
    






class CarBLE1:
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
    car = CarBLE1("EC:B1:B6:05:9D:50")
    
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
    # asyncio.run(ble_main())
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
