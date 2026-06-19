from meshlib import mrmeshpy as mm
from meshlib import mrmeshnumpy as mn
import numpy as np
from voxypy.models import Entity
import matplotlib.pyplot as plt
import pydicom
from pydicom.dataset import Dataset, FileDataset
from pydicom.uid import ExplicitVRLittleEndian
import os
from datetime import datetime

mesh = mm.loadMesh("calibr_model_navig_v1.stl")

# Проверка загрузки
print("Вершин:", mesh.points.size())
valid_faces = mesh.topology.getValidFaces()
print("Треугольников:", valid_faces.count())

if valid_faces.count() == 0:
    raise RuntimeError("Сетка не содержит треугольников! Проверьте файл.")
voxel_size = mm.suggestVoxelSize(mesh, 1e7)
origin_and_dims = mm.calcOriginAndDimensions(mesh.computeBoundingBox(), voxel_size)
print("origin:", origin_and_dims.origin)
print("dimensions:", origin_and_dims.dimensions)
print("voxel size:", voxel_size)
params = mm.MeshToDistanceVolumeParams()

# Set volume properties on the nested .vol object
params.vol.origin = origin_and_dims.origin
params.vol.dimensions = origin_and_dims.dimensions
params.vol.voxelSize = mm.Vector3f.diagonal(voxel_size) 

# Distance-specific parameters
params.dist.maxDistSq = (3 * voxel_size) ** 2
params.dist.nullOutsideMinMax = False

# Convert to volume
distance_volume = mm.meshToDistanceVolume(mesh, params)


np_arr = mn.getNumpy3Darray(distance_volume)

# Создадим бинарную маску: например, покажем вокселы внутри модели (расстояние <= 0)
"""# или вокселы вблизи поверхности (abs(distance) < threshold)
mask = np_arr <= 0  # Маска внутренних вокселов

# Для ускорения можно прореживать массив, если он слишком большой
# Например, взять каждый 2-й воксел: mask = mask[::2, ::2, ::2]

# Найдём координаты вокселов, попадающих в маску
filled_voxels = np.argwhere(mask)

if filled_voxels.size > 0:
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    # Отображаем только часть вокселов, чтобы не перегружать график
    # Например, каждые 10-й: filled_voxels[::10]
    ax.scatter(filled_voxels[::10, 0], filled_voxels[::10, 1], filled_voxels[::10, 2], 
               c='blue', marker='s', s=1)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.title('Воксельная модель')
    plt.show()
else:
    print("Нет вокселов, удовлетворяющих условию маски.")"""



# Ваш массив и параметры
# np_arr - 3D массив, полученный из distance_volume
# origin_and_dims.origin - mm.Vector3f (координаты начала сетки)
# voxel_size - float (размер вокселя, одинаковый по всем осям, либо можно взять отдельно)

# Преобразуем массив в порядок (Z, Y, X) для срезов по оси Z
volume = np_arr  # предположим, что исходный массив (X, Y, Z)
if volume.shape[0] != volume.shape[2]:  # если не (Z,Y,X), переставляем
    volume = np.transpose(volume, (2, 1, 0))  # теперь (Z, Y, X)

# Достаём параметры
origin = origin_and_dims.origin
vox_size = voxel_size  # float, если изометрический, либо Vector3f

# 1. Создаём бинарную маску: внутри (dist <= 0) → 255, снаружи → 0
# 1. Создаём бинарную маску: внутри (dist <= 0) → 255, снаружи → 0
mask = (np_arr <= 0).astype(np.uint8) * 255

# 2. Переупорядочиваем для срезов по оси Z (если нужно)
volume = mask
if volume.shape[0] != volume.shape[2]:   # если порядок не (Z, Y, X)
    volume = np.transpose(volume, (2, 1, 0))

# 3. Извлекаем параметры сетки
origin = origin_and_dims.origin
vox_size = voxel_size
if hasattr(vox_size, 'x'):
    vox_x, vox_y, vox_z = vox_size.x, vox_size.y, vox_size.z
else:
    vox_x = vox_y = vox_z = vox_size

# 4. Папка для сохранения
output_dir = "dicom_series_mask"
os.makedirs(output_dir, exist_ok=True)

study_uid = pydicom.uid.generate_uid()
series_uid = pydicom.uid.generate_uid()

file_meta = Dataset()
file_meta.MediaStorageSOPClassUID = '1.2.840.10008.5.1.4.1.1.2'  # CT
file_meta.MediaStorageSOPInstanceUID = pydicom.uid.generate_uid()
file_meta.TransferSyntaxUID = ExplicitVRLittleEndian

num_slices = volume.shape[0]

print(f"Сохранение {num_slices} срезов...")

for i in range(num_slices):
    slice_2d = volume[i, :, :]   # (Y, X)

    """# --- Визуальная проверка каждого 10-го среза ---
    if i % 10 == 0:
        plt.figure()
        plt.imshow(slice_2d, cmap='gray', vmin=0, vmax=255)
        plt.title(f'Slice {i} (min={slice_2d.min()}, max={slice_2d.max()})')
        plt.colorbar()
        #plt.savefig(os.path.join(output_dir, f"check_slice_{i:04d}.png"))
        plt.close()
        print(f"  → Сохранён проверочный PNG для среза {i}")"""

    # --- Создание DICOM-датасета ---
    ds = FileDataset(None, {}, file_meta=file_meta, preamble=b"\0"*128)

    ds.SOPClassUID = '1.2.840.10008.5.1.4.1.1.2'
    ds.SOPInstanceUID = pydicom.uid.generate_uid()
    ds.SeriesInstanceUID = series_uid
    ds.StudyInstanceUID = study_uid
    ds.FrameOfReferenceUID = pydicom.uid.generate_uid()

    ds.PatientName = "Anonymous"
    ds.PatientID = "123456"
    ds.Modality = "CT"

    ds.Rows, ds.Columns = slice_2d.shape
    ds.PixelSpacing = [float(vox_x), float(vox_y)]
    ds.SliceThickness = float(vox_z)

    # Положение верхнего левого пикселя
    ds.ImagePositionPatient = [
        float(origin.x),
        float(origin.y),
        float(origin.z + i * vox_z)
    ]
    # Ориентация (аксиальная)
    ds.ImageOrientationPatient = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0]

    ds.InstanceNumber = i + 1

    # Пиксельные данные (8‑бит без знака)
    pixel_array = slice_2d.astype(np.uint8)
    ds.PixelData = pixel_array.tobytes()
    ds.BitsAllocated = 8
    ds.BitsStored = 8
    ds.HighBit = 7
    ds.PixelRepresentation = 0   # unsigned
    ds.SamplesPerPixel = 1
    ds.PhotometricInterpretation = "MONOCHROME2"

    # Окно для рендеринга (чтобы 0 → чёрный, 255 → белый)
    ds.WindowCenter = [127.5]
    ds.WindowWidth = [255.0]
    ds.RescaleIntercept = 0.0
    ds.RescaleSlope = 1.0

    # Сохраняем файл
    filename = os.path.join(output_dir, f"slice_{i+1:04d}.dcm")
    ds.save_as(filename)
    if (i+1) % 50 == 0:
        print(f"  Сохранено {i+1}/{num_slices}")

print("Сохранение завершено.")

# ============================================================
# ДИАГНОСТИКА: читаем первый сохранённый файл и проверяем
# ============================================================
test_file = os.path.join(output_dir, "slice_0070.dcm")
if os.path.exists(test_file):
    print("\n Проверка первого файла:")
    ds_test = pydicom.dcmread(test_file)
    print(f"  PhotometricInterpretation : {ds_test.PhotometricInterpretation}")
    print(f"  WindowCenter              : {ds_test.WindowCenter}")
    print(f"  WindowWidth               : {ds_test.WindowWidth}")
    print(f"  RescaleIntercept          : {ds_test.RescaleIntercept}")
    print(f"  RescaleSlope              : {ds_test.RescaleSlope}")
    print(f"  PixelRepresentation       : {ds_test.PixelRepresentation}")
    print(f"  BitsAllocated             : {ds_test.BitsAllocated}")
    print(f"  Rows, Columns             : {ds_test.Rows}, {ds_test.Columns}")

    pixel_array_test = ds_test.pixel_array
    print(f"  Pixel array shape         : {pixel_array_test.shape}")
    print(f"  min value                 : {pixel_array_test.min()}")
    print(f"  max value                 : {pixel_array_test.max()}")
    unique_vals = np.unique(pixel_array_test)
    print(f"  unique values             : {unique_vals}")

    # Покажем изображение из сохранённого файла
    plt.figure(figsize=(8,8))
    plt.imshow(pixel_array_test, cmap='gray', vmin=0, vmax=255)
    plt.title("Первый срез из сохранённого DICOM")
    plt.colorbar()
    plt.show()
else:
    print("Файл не найден!")