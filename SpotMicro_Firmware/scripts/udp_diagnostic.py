import socket
import sys
import time

# Configuration
UDP_IP = "0.0.0.0"  # Listen on all interfaces
UDP_PORT = 8888      # Common Micro-ROS port

def start_listener():
    print(f"--- Network Connectivity Diagnostic Utility ---")
    print(f"[*] Starting UDP listener on {UDP_IP}:{UDP_PORT}")
    
    try:
        # Create a UDP socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((UDP_IP, UDP_PORT))
        print(f"[+] Successfully bound to port {UDP_PORT}")
    except Exception as e:
        print(f"[!] FATAL ERROR: Could not bind to port {UDP_PORT}. Error: {e}")
        print("[?] Tip: Check if another process is using this port (e.g., Micro-ROS Agent).")
        sys.exit(1)

    print(f"[*] Waiting for data from ESP32...")
    print(f"[*] Press Ctrl+C to stop.")

    while True:
        try:
            data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
            timestamp = time.strftime('%H:%M:%S')
            print(f"[{timestamp}] RECEIVED: '{data.decode('utf-8', errors='ignore')}' from {addr[0]}:{addr[1]}")
            
            # Send an acknowledgement back
            ack_msg = f"ACK: Received your message at {timestamp}"
            sock.sendto(ack_msg.encode(), addr)
            print(f"[{timestamp}] SENT ACK back to {addr[0]}")
            
        except KeyboardInterrupt:
            print("\n[*] Stopping listener...")
            sock.close()
            break
        except Exception as e:
            print(f"[!] ERROR during receive/send: {e}")

if __name__ == "__main__":
    start_listener()
