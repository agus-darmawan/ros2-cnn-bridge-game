# LITERALLY MIRIP KEK CreateDataSet.py
# Bedanya ini kita ngeload modelnya

import cv2
import numpy as np
import copy
import ModulKlasifikasiCitraCNN as mCNN

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Cannot open camera")
    exit()

LabelKelas = (
    "Kartu Tutup",
    "Keriting Dua",
    "Keriting Tiga",
    "Keriting Empat",
    "Keriting Lima",
    "Keriting Enam",
    "Keriting Tujuh",
    "Keriting Delapan",
    "Keriting Sembilan",
    "Keriting Sepuluh",
    "Keriting Jack",
    "Keriting Queen",
    "Keriting King",
    "Keriting Ace",
    # Lagi2 aku tutup soalnya blm semuanya
    # "Hati Dua",
    # "Hati Tiga",
    # "Hati Empat",
    # "Hati Lima",
    # "Hati Enam",
    # "Hati Tujuh",
    # "Hati Delapan",
    # "Hati Sembilan",
    # "Hati Sepuluh",
    # "Hati Jack",
    # "Hati Queen",
    # "Hati King",
    # "Hati Ace",
    # "Wajik Dua",
    # "Wajik Tiga",
    # "Wajik Empat",
    # "Wajik Lima",
    # "Wajik Enam",
    # "Wajik Tujuh",
    # "Wajik Delapan",
    # "Wajik Sembilan",
    # "Wajik Sepuluh",
    # "Wajik Jack",
    # "Wajik Queen",
    # "Wajik King",
    # "Wajik Ace",
    # "Sekop Dua",
    # "Sekop Tiga",
    # "Sekop Empat",
    # "Sekop Lima",
    # "Sekop Enam",
    # "Sekop Tujuh",
    # "Sekop Delapan",
    # "Sekop Sembilan",
    # "Sekop Sepuluh",
    # "Sekop Jack",
    # "Sekop Queen",
    # "Sekop King",
    # "Sekop Ace",
)

# Kita load modelnya
model = mCNN.LoadModel("model.h5")

# Fungsi dari pak Akok
def DrawText(img,sText,pos):
    font        = cv2.FONT_HERSHEY_SIMPLEX
    posf        = pos
    fontScale   = .7
    fontColor   = (0,0,255)
    thickness   = 2
    lineType    = 2
    cv2.putText(img,sText,
        posf,
        font,
        fontScale,
        fontColor,
        thickness,
        lineType)
    return copy.deepcopy(img)

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()

    # if frame is read correctly ret is True
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break
    
    
    # Nah dibagian Image processing ini sama persis kek CreateDataSet.py
    
    
    # Image processing
    # ======= 1. ======= Pertama, gambar kita buat grayscale dulu
    # Gray
    imGray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # cv2.imshow("1. Gray scale dulu", imGray)

    # ======= 2. ======= Kedua, kita threshold buat ambil
    # Threshold
    #                                                    Nilai 71 dan 10 bisa diatur sesuai kebutuhan masing2. Caranya? Cari aja di google 71 itu apa 10 itu apa. Kalo udah coba2 nilai yg pas buat kamera dan kartu kalian
    # NOTES:                                                                                        v   v    
    imThres = cv2.adaptiveThreshold(imGray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,71,10)
    # cv2.imshow("2. Adaptive thres", imThres)

    # ======= 3. ======= Next, kita ambil component yg connected.
    # Ini diambil dari contoh nya pak Akok di catatan buat HSV, cuma diedit dikit2
    # https://drive.google.com/file/d/1nuPMCajNSBYXBYO4t5nESIi74eNIzb1b/view
    # Connecting
    totalLabels, label_ids, values, centroid = cv2.connectedComponentsWithStats(imThres, 4, cv2.CV_32S)
    # Big index adalah array untuk menampung index mana yg luas area connected componentnya sesuai keinginan kita
    bigIndex = []
    for i in range(totalLabels):
        hw = values[i,2:4]
        # 100 dan 300 itu untuk cari widht diantara 100-300, 
        # 300 dan 500 itu cari height antara 300-500
        # Harus dicoba2 biar hasilnya sesuai dengan kartu kalian
        # Cara nyoba gimana? Kalo gambar kartunya belom di kotakin, berarti nilainya masih salah. Tweeking aja coba2
        if (100<hw[0]<300 and 300<hw[1]<500):
            bigIndex.append(i)

    # ======= 4. ======= Check, apakah ada connected component yg sesuai dengan luas yg kita define
    # Kalo ada kita kotakin trus kita kotakin
    for i in bigIndex:
        topLeft = values[i,0:2]
        bottomRight = values[i,0:2]+values[i,2:4]
        # v                     v       Disini aku ngotakin di gambar asli 'frame'
        frame = cv2.rectangle(frame, topLeft, bottomRight, color=(0,0,255), thickness=3)

        # Disini ada break, yg berarti kita cuma ngambil 1 item doang
        break
    # Trus tampilin
    # cv2.imshow("4. Hasil habis dikotakin", frame)
    
    # ======= 5. ======= Kita crop yg dikotakin tadi
    for i in bigIndex:
        topLeft = values[i,0:2]
        bottomRight = values[i,0:2]+values[i,2:4]
        
        # Ini buat ngecrop gambarnya
        cardImage = imThres[topLeft[1]:bottomRight[1],topLeft[0]:bottomRight[0]]
        
        # Lanjut ditampilin
        cv2.imshow('5. Hasil dari cardImage', cardImage)
        # Gambar ini yg bakal disimpen dan masuk jadi dataset model

        # Nah disini bedanya.
        # cardImage kan udah memuat gambar kartunya, ini kita masukin ke modelnya
        # Caranya kek gini, dan ini juga diambil dari modul bapaknya

        # CardImage dalam kasusku adalah gambar grayscale
        # Sedangkan gambar yg dimasukin ke model diubah ke BGR (RGB)
        # Jadi harus kita convert balik dulu ke BGR
        cardImage = cv2.cvtColor(cardImage, cv2.COLOR_GRAY2BGR)

        # Feed into model
        X = []
        #                    v     dimasukin disini
        img = cv2.resize(cardImage,(128,128))
        img = np.asarray(img)/255
        img = img.astype('float32')
        X.append(img)
        X = np.array(X)
        X = X.astype('float32')

        # Predict
        hs = model.predict(X,verbose = 0)
        n = np.max(np.where(hs== hs.max()))

        # Put text into image
        textCoordinate = topLeft + [0, -10]
        DrawText(frame, f'{LabelKelas[n]} {"{:.2f}".format(hs[0,n])}', textCoordinate - [95,0])

    cv2.imshow('Result', frame)
    
    if cv2.waitKey(1) == 27:
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()