# Perbaikan Trimesh-Trimesh Contact Hash Table Overflow

Dokumen ini menjelaskan perubahan yang dilakukan untuk mengatasi error:
```
ODE Message 2: Trimesh-trimesh contact hash table bucket overflow - close contacts might not be culled in AddContactToNode()
```

## Masalah

Error ini terjadi karena ODE (Open Dynamics Engine) kehabisan kapasitas hash table internal untuk mendeteksi collision antar mesh. Proyek ini memiliki:
- 36 roller (9 per roda × 4 roda) dengan collision mesh kompleks
- 4 roda dengan collision mesh
- 1 base dengan collision mesh
- 1 map dengan collision mesh DAE

Total terlalu banyak trimesh collision yang berinteraksi secara bersamaan.

## Solusi yang Diterapkan

### 1. Update Physics Parameters (empty.world)
**File:** `src/mecanumbot_description/world/empty.world`

**Perubahan:**
- Menambahkan ODE solver configuration dengan parameter:
  - `<iters>50</iters>` - Iterasi solver ditingkatkan
  - `<sor>1.3</sor>` - Successive Over-Relaxation
  - `<contact_max_correcting_vel>100</contact_max_correcting_vel>`
  - `<contact_surface_layer>0.001</contact_surface_layer>`

- Menambahkan surface properties pada map collision untuk mengontrol kontak

### 2. World File Optimized Baru
**File:** `src/mecanumbot_description/world/optimized.world`

**Fitur:**
- Physics configuration yang lebih agresif:
  - `<iters>100</iters>` - Iterasi solver lebih tinggi
  - Ground plane dengan collision sederhana (plane geometry)
  - Scene settings untuk lighting dan shadows

**Cara menggunakan:**
```bash
# Ganti di launch file
# Dari:
world_file = 'empty.world'
# Ke:
world_file = 'optimized.world'
```

### 3. Optimasi Collision Shape

#### Roller (roller.xacro)
**File:** `src/mecanumbot_description/urdf/parts/roller/roller.xacro`

**Perubahan:**
- Collision: Mesh STL → Cylinder primitive
- Radius: 0.006m, Length: 0.028m
- Mengurangi jumlah trimesh collision dari ribuan triangle menjadi 1 primitive shape

**Dampak:**
- 36 roller × kompleksitas mesh → 36 cylinder sederhana
- Pengurangan drastis beban ODE

#### Mecanum Wheel (mecanum_wheel.xacro & mecanum_wheel2.xacro)
**File:**
- `src/mecanumbot_description/urdf/parts/mecanum_wheel/mecanum_wheel.xacro`
- `src/mecanumbot_description/urdf/parts/mecanum_wheel2/mecanum_wheel2.xacro`

**Perubahan:**
- Collision: Mesh STL → Cylinder primitive
- Radius: 0.0485m, Length: 0.036m

**Dampak:**
- 4 roda × kompleksitas mesh → 4 cylinder sederhana

#### Base (base.xacro)
**File:** `src/mecanumbot_description/urdf/parts/base/base.xacro`

**Perubahan:**
- Collision: Mesh STL → Box primitive
- Size: 0.3302m × 0.3302m × 0.102m

**Dampak:**
- Base mesh kompleks → 1 box sederhana

### 4. Optimasi Contact Properties

Semua collision sekarang menggunakan parameter:
```xml
<surface>
  <contact>
    <ode>
      <max_vel>10</max_vel>
      <min_depth>0.0001</min_depth>
    </ode>
  </contact>
</surface>
```

Parameter `min_depth` membantu mengurangi jumlah kontak yang terdeteksi.

## File Backup

File-file berikut dibackup sebelum perubahan:
- `roller.xacro.backup`
- `mecanum_wheel.xacro.backup`
- `mecanum_wheel2.xacro.backup`
- `base.xacro.backup`

Untuk restore:
```bash
cp roller.xacro.backup roller.xacro
cp mecanum_wheel.xacro.backup mecanum_wheel.xacro
cp mecanum_wheel2.xacro.backup mecanum_wheel2.xacro
cp base.xacro.backup base.xacro
```

## Testing

Setelah rebuild:
```bash
colcon build --packages-select mecanumbot_description
source install/setup.bash
ros2 launch mecanumbot_bringup simulation.launch.py world:=optimized.world
```

## Hasil yang Diharapkan

1. Tidak ada lagi warning "trimesh-trimesh contact hash table bucket overflow"
2. Simulasi lebih stabil
3. Performance lebih baik (FPS lebih tinggi)
4. Robot tidak lagi "bergetar" atau glitch akibat multiple collision responses

## Catatan Penting

- Visual tetap menggunakan mesh DAE (tidak berubah)
- Hanya collision shape yang dioptimasi
- Jika presisi collision sangat penting, pertimbangkan untuk:
  - Menggunakan convex hull sederhana
  - Menggabungkan beberapa primitive shapes
  - Menggunakan trimesh dengan polygon count lebih rendah