import cv2 as cv

# Abre a câmera 1 usando Media Foundation
cap = cv.VideoCapture(1, cv.CAP_MSMF)

if not cap.isOpened():
    print("Erro ao acessar a câmera!")
    exit()

while True:
    ret, frame = cap.read()

    if not ret:
        print("Erro ao capturar imagem!")
        break

    cv.imshow("Camera do Celular - DroidCam", frame)

    if cv.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv.destroyAllWindows()