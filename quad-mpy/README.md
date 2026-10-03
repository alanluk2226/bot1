# Quad robot MicroPython code

## Links
- **Arduino Wi-Fi version of this kit**: [repository](https://github.com/AniPython/quad-arduino-wifi)
- **Bilibili**: [video](https://b23.tv/kzp9yXQ)

## Wi-Fi settings in `main.py`

### 1. IP address
Match the address to your router:

```python
ifconfig = ("192.168.x.182", "255.255.255.0", "192.168.x.1", "8.8.8.8")
```

Replace `x` with the router's subnet. On Windows: Settings, Network and Internet, Ethernet, then read the IPv4 address and DNS server.

Example:

```
IPv4 address: 192.168.2.10
IPv4 DNS server: 192.168.2.1
```

Then `ifconfig` is:

```
ifconfig = ("192.168.2.182", "255.255.255.0", "192.168.2.1", "8.8.8.8")
```

Open `192.168.2.182` in a browser on the computer or phone to see the control page.

### 2. Wi-Fi name and password

```
robot_wifi.create_connect_route(ssid='wifi name', password='wifi password', ifconfig=ifconfig)
```
