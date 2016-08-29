# Robot PenguIn

## Configuration

### Login

**User:** pi
**Pass:** raspberry

### Setup WiFi/WLAN

* Scan for available WiFi networks
```
sudo iwlist wlan0 scan
```

* Edit /etc/wpa\_supplicant/wpa\_supplicant.conf
```
sudo vim /etc/wpa_supplicant/wpa_supplicant.conf
```

* Go to the bottom of the file and add the following:
```
network={
    ssid="The_ESSID_from_earlier"
    psk="Your_wifi_password"
}
```

* Restart the WiFi adapter
```
sudo ifdown wlan0
sudo ifup wlan0
```

* Verify that a connection was erstablished
```
ifconfig wlan0
```

