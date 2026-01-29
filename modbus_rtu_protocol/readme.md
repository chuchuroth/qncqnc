
```
git clone -b modbus_rtu_protocol --single-branch  
cd modbus_rtu_protocol/modbus_rtu_protocol
```


Or checkout in existing repo:
```
git fetch gitlab modbus_rtu_protocol
git checkout modbus_rtu_protocol
```

Docker Build:
```
cd modbus_rtu_protocol
./scripts/build.sh
./build/modbus_rtu_example /dev/ttyUSB0 1 115200
```


Docker Run:
```
docker run --rm -it --device=/dev/ttyUSB0 modbus-rtu-protocol:latest /dev/ttyUSB0 1 115200
```

Docker Compose:
```
cd modbus_rtu_protocol
docker-compose up modbus-rtu
```