# Generalized CVRP Solver — AI Hackathon: Route Optimization 2026

ทีม เทสเทสหนึ่งสองสามสี่

แพ็กเกจนี้รวมโค้ด วิธีรัน และไฟล์ข้อมูลที่จำเป็นสำหรับ **Mission 2 (Benchmark ทั้ง 3 ชุด)** และ **Mission 3 (Self-created Case: CPAC Ready-Mix Concrete Delivery)** ทุกตัวเลขในภาพหลักฐานและในสไลด์ ตรวจสอบย้อนกลับได้โดยการรันโค้ดในซิปนี้

## บทนำ

ตัวแก้ปัญหา CVRP แบบทั่วไป (เขียนด้วย Python ล้วน ไม่ฝังเส้นทางหรือคำตอบไว้ล่วงหน้า) สำหรับรายการเเข่ง Generalized CVRP Solver — AI Hackathon: Route Optimization 2026
**

- Benchmark: A-n32-k5 **5.48%**, B-n31-k5 **0.74%**, P-n40-k5 **12.23%** (ค่า optimal)
- เคสที่ทีมสร้างเองด้านการส่งคอนกรีตผสมเสร็จของ SCG-CPAC: ระยะทางรวม **247 กม.** **สั้นกว่า 16.6%** เมื่อเทียบกับ Nearest Neighbor และทุกการส่งมอบอยู่ภายในเวลาความสด 90 นาที
- 🎬 [วิดีโอสาธิต](https://www.youtube.com/watch?v=PIdKXsvkfu0&t=3s) · 💻 [Repo](https://github.com/PhuwanaiMark24/AI-Hackathon-Route-Optimization-2026)

![ผลลัพธ์ CPAC](https://raw.githubusercontent.com/<your-username>/ai-for-routing-cvrp/main/results/evidence/Evidence_CPAC-n16-k8-fresh.png)
![ผลลัพธ์ CPAC](https://github.com/PhuwanaiMark24/AI-Hackathon-Route-Optimization-2026/Benchmark_and_Solver/data_Self-created_Case/Evidence_CPAC-n16-k8-fresh)

---

## 1. โครงสร้างไฟล์

```
Benchmark & Solver/
├── README.md                          ไฟล์นี้
├── Route_Optimizer.ipynb              Notebook รวมทุกอย่างไว้ไฟล์เดียว (Mission 2 + 3)
├── data 3 Benchmark/                  ► Mission 2
│   ├── vrp_solver.py                  Solver ตัวเดียว (Generalized CVRP Solver)
│   ├── A-n32-k5.vrp                   ข้อมูลโจทย์ Benchmark 1
│   ├── B-n31-k5.vrp                   ข้อมูลโจทย์ Benchmark 2
│   └── P-n40-k5.vrp                   ข้อมูลโจทย์ Benchmark 3
└── data Self-created Case/            ► Mission 3
    ├── vrp_solver_case3.py            Solver ของ Mission 3 (รันได้ด้วยตัวเอง)
    ├── CPAC-n16-k8-fresh.txt          ข้อมูลโจทย์ที่ทีมสร้างเอง (มีกติกาความสด)
    ├── CPAC_Case_Summary.xlsx         ตารางสรุป Case (เส้นทาง, ตรวจสอบ, เปรียบเทียบ, CO2)
    └── Self-created_Case_Context&Result.pdf   เอกสารอธิบาย Case
```

| ไฟล์ | ใช้ทำอะไร |
|---|---|
| `vrp_solver.py` | **Solver ของ Mission 2** ใช้ฟังก์ชันเดียวกันกับทุกไฟล์ข้อมูล ตรวจแค่เรื่องความจุรถ |
| `vrp_solver_case3.py` | **Solver ของ Mission 3** ข้างในมี 2 ส่วน: PART A คือโค้ด Mission 2 ที่คัดลอกมาทั้งชุดโดยไม่แก้ไข, PART B คือส่วนที่เพิ่ม (กติกาความสด 90 นาที, แยกคันที่สาย, Nearest Neighbor, exact optimum, CO2, sensitivity) **รันได้โดยไม่ต้องมี `vrp_solver.py`** |
| `Route_Optimizer.ipynb` | โค้ดชุดเดียวกันรวมไว้ใน notebook พร้อมผลรัน กราฟ และคำอธิบายภาษาไทย ไม่ import ไฟล์ `.py` |
| ไฟล์ `.vrp` / `.txt` | ข้อมูลโจทย์ในรูปแบบ VRPLIB (พิกัด, ปริมาณที่ต้องส่ง, ความจุ) ไม่มีคำตอบหรือเส้นทางอยู่ในไฟล์ |

> ไฟล์ `CPAC-n16-k8-fresh.txt` มีบรรทัดเพิ่ม 3 บรรทัดจากรูปแบบมาตรฐาน คือ `FRESHNESS_LIMIT_MIN : 90`, `SPEED_KMH : 30`, `SERVICE_TIME_MIN : 10` เพื่อบอก Solver ของ Mission 3 ว่าคอนกรีตต้องถึงไซต์ภายใน 90 นาที ถ้าไม่มี 3 บรรทัดนี้ Solver จะไม่ตรวจเวลา และจะได้คำตอบ 241 กม. (ตรวจเฉพาะความจุ) แทน 247 กม.

---

## 2. สภาพแวดล้อมที่ใช้

- **Python 3** (พัฒนาและทดสอบด้วย Python 3.12.3, ใช้ได้กับ 3.8 ขึ้นไป)
- **ไลบรารีภายนอก: ไม่จำเป็นสำหรับตัว Solver** — `vrp_solver.py` และ `vrp_solver_case3.py` ใช้เฉพาะไลบรารีมาตรฐานของ Python (`math`, `os`, `re`, `itertools`, `dataclasses`, `functools`, `typing`)
- **matplotlib** (ทดสอบกับ 3.10.8) ใช้เฉพาะตอนวาดกราฟใน notebook (Google Colab มีติดตั้งให้แล้ว)
- **Random seed: ไม่มี** — อัลกอริทึมเป็นแบบ deterministic (Clarke-Wright Savings + Relocate + Swap + 2-opt) ใส่ข้อมูลเดิมได้ผลเดิมทุกครั้ง จึงไม่มีค่า seed ให้ระบุ
- **Parameter:** ไม่มีค่าที่ต้องปรับ สิ่งที่โค้ดอ่านจากไฟล์ข้อมูลคือความจุรถ จำนวนรถ และค่า Optimal ที่ตีพิมพ์ (ใช้ตรวจ Fleet Limit และคำนวณ Gap เท่านั้น) ส่วน Mission 3 อ่านเพิ่ม 3 ค่าคือเวลาจำกัด (90 นาที), ความเร็ว (30 กม./ชม.), เวลาเทต่อไซต์ (10 นาที)

---

## 3. วิธีรัน

เปิด terminal แล้วเข้าไปที่โฟลเดอร์ `Benchmark & Solver` (ชื่อโฟลเดอร์มีช่องว่าง ต้องใส่เครื่องหมายคำพูดเวลาพิมพ์ path) ถ้าใช้ Windows ให้พิมพ์ `python` หรือ `py` แทน `python3`

### Mission 2 — Benchmark 3 ชุด

```bash
cd "data 3 Benchmark"

python3 vrp_solver.py A-n32-k5.vrp
python3 vrp_solver.py B-n31-k5.vrp
python3 vrp_solver.py P-n40-k5.vrp
```

รันคำสั่งเดียวให้ครบทั้ง 3 ไฟล์พร้อมตารางสรุปได้ด้วย `python3 vrp_solver.py` (ไม่ใส่ชื่อไฟล์ โปรแกรมจะรันทุกไฟล์ `.vrp` ในโฟลเดอร์)

แต่ละคำสั่งจะพิมพ์ **Result Card** ซึ่งประกอบด้วย ชื่อ Instance, เส้นทางของรถทุกคัน, Total Distance, ค่า Optimal ที่ตีพิมพ์, Gap (%), สถานะ Feasible และผลตรวจกติกา 5 ข้อ (PASS/FAIL)

### Mission 3 — Self-created Case (CPAC)

```bash
cd "data Self-created Case"

python3 vrp_solver_case3.py CPAC-n16-k8-fresh.txt
```

จะพิมพ์เส้นทางของรถแต่ละคัน (พร้อมเวลาที่ไปถึงไซต์สุดท้าย), Total Distance, ผลตรวจกติกา 6 ข้อ, เปรียบเทียบกับ exact optimum และ Nearest Neighbor, ปริมาณ CO2, ผลของแต่ละขั้นตอน (เฉพาะความจุ → แยกคันที่สาย → ปรับปรุงแบบรู้เวลา) และตาราง sensitivity

### Notebook (รันทุกอย่างในไฟล์เดียว)

เปิด `Route_Optimizer.ipynb` ด้วย Google Colab, Jupyter หรือ VS Code แล้วสั่ง **Run all**

- ถ้ามีปุ่มให้อัปโหลดไฟล์ จะเลือกไฟล์ข้อมูลจากโฟลเดอร์ทั้งสองมาอัปโหลดก็ได้ หรือกด Cancel เพื่อใช้ข้อมูลโจทย์ที่ฝังไว้ในเซลล์ (มีเฉพาะพิกัด demand และความจุ ไม่มีเส้นทางหรือคำตอบ)
- ผลรัน (เส้นทาง ระยะทาง กราฟ) ถูกบันทึกอยู่ใน notebook แล้ว เปิดอ่านได้โดยไม่ต้องรัน

---

## 4. ผลที่ต้องได้ (ต้องตรงกับภาพหลักฐานและข้อมูลที่กรอกในฟอร์ม)

**Mission 2**

| Instance | Total Distance | Optimal (อ้างอิง) | Gap | จำนวนรถ | Feasible |
|---|---|---|---|---|---|
| A-n32-k5 | **827** | 784 | 5.48% | 5 | YES |
| B-n31-k5 | **677** | 672 | 0.74% | 5 | YES |
| P-n40-k5 | **514** | 458 | 12.23% | 5 | YES |

**Mission 3 (CPAC-n16-k8-fresh.txt)**

| รายการ | ผล |
|---|---|
| ระยะทางรวม | **247 กม.** |
| จำนวนรถที่ใช้ | 7 จาก 8 คัน |
| Feasible | YES ผ่านครบ 6 กติกา (5 ข้อของ CVRP + ความสด 90 นาที) |
| ถึงไซต์ช้าที่สุด | 84 นาที |
| เทียบ exact optimum | 247 กม. (Gap 0.00%) |
| เทียบ Nearest Neighbor | 296 กม. / 8 คัน (คำตอบของทีมสั้นกว่า 16.6%) |
| เฉพาะความจุ (ไม่ตรวจเวลา) | 241 กม. แต่ **ไม่ Feasible** เพราะมีเส้นทางถึงไซต์ 13 ที่นาทีที่ 94 |
| CO2 ต่อรอบจ่ายงาน | 231.7 kg (สมมติฐาน 0.35 ลิตร/กม. × 2.68 kg CO2/ลิตร) |

---

## 5. อัลกอริทึม

Solver ทำงาน 3 ขั้นตอน เหมือนกันทุกไฟล์:

1. **Clarke-Wright Savings** — เริ่มจาก 1 ไซต์ = 1 คัน แล้วรวมคู่ไซต์ที่ประหยัดระยะทางได้มากที่สุดก่อน (ไม่เกินความจุ)
2. **Local Search** วนซ้ำจนไม่มีอะไรดีขึ้น: **Relocate** (ย้ายไซต์ไปคันอื่น ถ้าคันเดิมว่างจะยุบคันนั้นทิ้ง) → **Swap** (สลับไซต์ข้ามคัน) → **2-opt** (แก้เส้นที่วิ่งไขว้กันในคันเดียว)
3. **ตรวจกติกา** (Node Coverage, Visited-once, Capacity, Fleet Limit, Depot Routing) แยกจากขั้นตอนสร้างเส้นทาง แล้วจึงคำนวณ Gap

ทุกขั้นตอนถามฟังก์ชัน `route_ok(เส้นทาง)` ว่าเส้นทางใหม่ใช้ได้หรือไม่ Mission 2 ตรวจแค่ความจุ ส่วน Mission 3 ส่ง `route_ok` ที่ตรวจ **ความจุ + เวลาถึงไซต์ ≤ 90 นาที** เข้าไปแทน โดยใช้ขั้นตอนเดิมทั้งหมด (`vrp_solver_case3.py` ส่วน PART A คือโค้ด Mission 2 ที่ไม่ได้แก้ไข)

---

## 6. ไม่มีการฝังคำตอบ (No hard-coding)

- โค้ดใช้ขั้นตอนเดียวกับทุกไฟล์: `อ่านไฟล์ → คำนวณระยะทาง → Clarke-Wright → Local Search → ตรวจกติกา → คำนวณ Gap` ไม่มีเงื่อนไขตรวจชื่อโจทย์ ไม่มีเส้นทางหรือคำตอบที่เขียนไว้ล่วงหน้า
- ข้อมูลที่อ่านจากไฟล์โจทย์ (จำนวนรถ ค่า Optimal ความจุ พิกัด) เป็นข้อมูลของโจทย์ ไม่ใช่คำตอบ
- ทดสอบกับไฟล์อื่นได้ทันที: `python3 vrp_solver.py ชื่อไฟล์.vrp`
- เปลี่ยนแค่ไฟล์ข้อมูลกับ `route_ok` ก็ได้ผลของ Mission 2 และ Mission 3 จากโค้ดชุดเดียวกัน

---

## 7. วิธีทำภาพหลักฐาน (1 Benchmark : 1 ภาพ)

1. รันคำสั่งของ Benchmark นั้นตามข้อ 3 (เช่น `python3 vrp_solver.py A-n32-k5.vrp`)
2. แคปหน้าจอ terminal ให้เห็น **บรรทัดคำสั่งที่พิมพ์และผลลัพธ์ทั้งหมด** ได้แก่ชื่อ Instance, เส้นทางทุกคัน และ Total Distance (ถ้ารันใน notebook ให้แคปเซลล์ที่มีทั้งโค้ดและผลรัน)
3. ตัดขอบหรือย่อขนาดได้ แต่ห้ามแก้ไข ปกปิด หรือเพิ่มข้อมูลในภาพ
4. ตรวจว่าตัวเลขตรงกับตารางในข้อ 4 ก่อนอัปโหลด

---

## 8. ไม่ได้รวมอยู่ในซิปนี้

ไม่มีสภาพแวดล้อมเสมือน (`.venv`), `node_modules`, รหัสผ่าน, API key หรือข้อมูลลับใดๆ
