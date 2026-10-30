# Docker Setup — รัน Ubuntu 22.04 (ROS2 Humble) ใน Docker

เครื่อง host เป็น **Ubuntu 24.04** แต่ต้องการ environment **Ubuntu 22.04 (Humble)**
→ ใช้ Docker สร้างกล่อง Ubuntu 22.04 แยกต่างหาก โดยไม่ยุ่งกับ host

---

## ส่วนที่ 1 — ติดตั้ง Docker (ครั้งเดียว)

### 1.1 ลง Docker
```bash
sudo apt update
sudo apt install -y docker.io
```

### 1.2 ใช้ docker ได้โดยไม่ต้อง sudo
```bash
sudo usermod -aG docker $USER
```
> ⚠️ ต้อง **logout แล้ว login ใหม่** (หรือ reboot) ครั้งเดียว ถึงจะมีผล

### 1.3 เช็คว่าใช้งานได้
```bash
docker run hello-world      # เห็นข้อความต้อนรับ = ใช้ได้
```

---

## ส่วนที่ 2 — องค์ประกอบ 2 ไฟล์

| ไฟล์ | หน้าที่ |
|------|--------|
| `docker/Dockerfile` | สูตรสร้าง image: เริ่มจาก `osrf/ros:humble-desktop` (Ubuntu 22.04) |
| `docker/run.sh` | สคริปต์ย่อคำสั่ง docker: `build / up / exec / down` |

---

## ส่วนที่ 3 — Build & Run

| คำสั่ง | ทำอะไร |
|--------|--------|
| `./docker/run.sh build` | สร้าง image จาก Dockerfile (ครั้งแรกครั้งเดียว, ใช้เวลานาน) |
| `./docker/run.sh up`    | เข้า container — mount `DART_ws`→`/ws`, ต่อ GUI + network |
| `./docker/run.sh exec`  | เปิด shell เพิ่มใน container เดิม (หลายหน้าต่าง) |
| `./docker/run.sh down`  | ปิด container |

### 3.1 Build image
```bash
cd ~/DART_ws
./docker/run.sh build
```

### 3.2 เข้า container
```bash
./docker/run.sh up
# prompt เปลี่ยนเป็น  root@<host>:/ws#  = อยู่ใน Ubuntu 22.04 แล้ว
```

### 3.3 เปิดหน้าต่างเพิ่ม (container เดิม)
```bash
./docker/run.sh exec
```

### 3.4 ปิดเมื่อเลิกใช้
```bash
./docker/run.sh down
```

---

## ส่วนที่ 4 — สิ่งที่ `run.sh` ตั้งให้อัตโนมัติตอน `up`

```
--network host        → container ใช้ network เดียวกับ host
--ipc host            → shared memory ระหว่าง host กับ container
-v DART_ws:/ws        → แก้ไฟล์ฝั่ง host ↔ container ร่วมกัน (โฟลเดอร์เดียวกัน)
-e DISPLAY + X11 mount → เปิดโปรแกรมมี GUI จาก container ขึ้นจอ host ได้
-e ROS_DOMAIN_ID=30
-e TURTLEBOT3_MODEL=burger
```

---

## ปัญหาที่เจอบ่อย

| อาการ | สาเหตุ / แก้ |
|-------|-------------|
| `docker: permission denied` | ยังไม่ได้ `usermod -aG docker $USER` + logout/login |
| ไฟล์ใน `DART_ws` ติดกุญแจ 🔒 แก้ไม่ได้ | Docker สร้างไฟล์เป็น root → `sudo chown -R $USER:$USER ~/DART_ws` |
| GUI ไม่ขึ้น (Gazebo/RViz) | ยังไม่ `xhost +local:root` (run.sh ทำให้แล้วตอน up) |
| build image นานมาก | ปกติ — ครั้งแรกต้องโหลด + ติดตั้งทั้ง stack |
