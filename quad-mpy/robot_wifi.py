
import machine
import usocket as socket
import network
import time
import json


class RobotWifi:

    def __init__(self, robot, html_path='index.html'):
        self.robot = robot
        try:
            with open(html_path, 'r', encoding='utf-8') as file:
                self.html = file.read()
        except OSError as e:
            assert False, f"missing index.html; upload that file too"

    def create_connect_ap(self, essid, password, ifconfig=None):
        """Access-point mode: the phone joins the ESP32 directly, with no router."""
        ap = network.WLAN(network.AP_IF)
        if ifconfig:
            ap.ifconfig(ifconfig)
        ap.active(True)
        ap.config(essid=essid, password=password, authmode=network.AUTH_WPA_WPA2_PSK)
        print('Access Point created!')
        machine.PWM(machine.Pin(2), duty=512)
        ip = ap.ifconfig()[0]
        print("ip:", ip)
        return ip

    def create_connect_route(self, ssid, password, ifconfig=None, timeout=12):
        """Station mode: the ESP32 and the phone both join the router."""
        wlan = network.WLAN(network.STA_IF)
        if ifconfig:
            # Fixed address. Without it, the router may hand out a new one each time.
            wlan.ifconfig(ifconfig)
        wlan.active(True)
        if not wlan.isconnected():
            print('connecting to network...')
            wlan.connect(ssid, password)
            i = 1
            while not wlan.isconnected():
                print("connecting...{}".format(i))
                i += 1
                time.sleep(1)
                if i > timeout:
                    raise OSError("Wi-Fi timed out. Check the name and password. The ESP32 joins 2.4 GHz only.")
        ip = wlan.ifconfig()[0]
        machine.PWM(machine.Pin(2), duty=512)
        print("ip:", ip)
        return ip

    def handle_post_request(self, post_data):
        try:
            command = json.loads(post_data).get("command")
        except ValueError as e:
            err = "JSON parse error: " + str(e)
            print(err)
            return json.dumps({"status": "400", "msg": err})
        
        if command:
            try:
                print(command)
                method = getattr(self.robot, command)
                method()
                return json.dumps({"status": "200", "msg": command})
            except Exception as e:
                err = "Error executing command:" + str(e)
                print(err)
                return json.dumps({"status": "500", "msg": err})
        else:
            return json.dumps({"status": "400", "msg": "missing command"})

    def handle_get_request(self):
        response_headers = 'HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n'
        return response_headers + self.html

    def handle_request(self, client_socket):
        request = client_socket.recv(1024)
        request_str = request.decode('utf-8')
        request_lines = request_str.split('\r\n')
        method, path, _ = request_lines[0].split()

        if method == "POST" and path == "/control":
            post_data = request_lines[-1]
            response = self.handle_post_request(post_data)

        else:
            response = self.handle_get_request()

        client_socket.send(response.encode('utf-8'))
        client_socket.close()

    # HTTP server
    def create_server(self):
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.bind(('', 80))
        server_socket.listen(128)
        print('HTTP server started!')

        while True:
            client_socket, addr = server_socket.accept()
            self.handle_request(client_socket)
