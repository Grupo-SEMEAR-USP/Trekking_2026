import cv2 as cv
import numpy as np

cap = cv.VideoCapture(1, cv.CAP_MSMF) # Inicializa a câmera 

while True:
    ret, frame = cap.read()

    if not ret:
        break

    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV) # Converte BGR para HSV 
    # Define o intervalo da cor amarela (pesquisei na net, mas dá pra alterar)
    amarelo_min = np.array([20, 100, 100])
    amarelo_max = np.array([35, 255, 255])
    mascara = cv.inRange(hsv, amarelo_min, amarelo_max) # Cria a máscara binária
    contornos, _ = cv.findContours(mascara,cv.RETR_EXTERNAL,cv.CHAIN_APPROX_SIMPLE) # Encontra os contornos

    for contorno in contornos:
        # Ignora regiões pequenas
        if cv.contourArea(contorno) > 15000:
            # Calcula o retângulo delimitador
            x, y, w, h = cv.boundingRect(contorno)
            # Desenha o retângulo
            cv.rectangle(frame,(x, y),(x + w, y + h),(0, 255, 0),2)
            # Calcula o centro do objeto
            centro_x = x + w // 2
            centro_y = y + h // 2
            # Marca o centro
            cv.circle(frame,(centro_x, centro_y),5,(0, 0, 255),-1)
            # Identifica o objeto
            cv.putText(frame,"Amarelo",(x, y - 10),cv.FONT_HERSHEY_SIMPLEX,0.7,(0, 255, 0),2)

    # Exibe a imagem e a máscara
    cv.imshow("Deteccao de Amarelo", frame)
    cv.imshow("Mascara", mascara)

    # Pressione Q para sair
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv.destroyAllWindows()