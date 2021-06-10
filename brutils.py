#Código para seguir a pista usando o cormudule
#----> try do roda todo frame
    try:
        temp_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
        mask = segmenta_linha_amarela(temp_image)
        media, centro, maior_area =  cormodule.identifica_cor(mask)#cormudule é o do projeto

#----> while do robo 
if(len(centro) > 0 and len(media) > 0):
    if (media[0] > centro[0]):
        vel = Twist(Vector3(velocidade,0,0), Vector3(0,0,-0.2))
    else:
        vel = Twist(Vector3(velocidade,0,0), Vector3(0,0,0.2))

##################### ARUCO #############################
#Para indentificar aruco - input imagem bgr
def acha_aruco(gray):
    gray = cv2.cvtColor(gray, cv2.COLOR_BGR2GRAY)
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    print(ids)

    if ids is not None:
        #-- ret = [rvec, tvec, ?]
        #-- rvec = [[rvec_1], [rvec_2], ...] vetor de rotação
        #-- tvec = [[tvec_1], [tvec_2], ...] vetor de translação
        ret = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)
        rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]
        #-- Desenha um retanculo e exibe Id do marker encontrado
        aruco.drawDetectedMarkers(temp_image, corners, ids) 
        aruco.drawAxis(temp_image, camera_matrix, camera_distortion, rvec, tvec, 1)
        #-- Print tvec vetor de tanslação em x y z
        str_position = "Marker x=%4.0f  y=%4.0f  z=%4.0f"%(tvec[0], tvec[1], tvec[2])
        print(str_position)
        cv2.putText(temp_image, str_position, (0, 100), font, 1, (0, 255, 0), 1, cv2.LINE_AA)
        ##############----- Referencia dos Eixos------###########################
        # Linha referencia em X
        cv2.line(temp_image, (int(temp_image.shape[1]/2),int(temp_image.shape[0]/2)), ((int(temp_image.shape[1]/2) + 50),(int(temp_image.shape[0]/2))), (0,0,255), 5) 
        # Linha referencia em Y
        cv2.line(temp_image, (int(temp_image.shape[1]/2),int(temp_image.shape[0]/2)), ((int(temp_image.shape[1]/2)),(int(temp_image.shape[0]/2) + 50)), (0,255,0), 5) 	
        
        #####################---- Distancia Euclidiana ----#####################
        # Calcula a distancia usando apenas a matriz tvec, matriz de tanslação
        # Pode usar qualquer uma das duas formas
        distance = np.sqrt(tvec[0]**2 + tvec[1]**2 + tvec[2]**2)
        distancenp = np.linalg.norm(tvec)
        #-- Print distance
        str_dist = "Dist aruco=%4.0f  dis.np=%4.0f"%(distance, distancenp)
        print(str_dist)
        cv2.putText(temp_image, str_dist, (0, 15), font, 1, (0, 255, 0), 1, cv2.LINE_AA)

        return distance, ids
    return 1000,[[-1]]
#Variaveis que essa função precisa
aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
parameters  = aruco.DetectorParameters_create()
parameters.minDistanceToBorder = 0
parameters.adaptiveThreshWinSizeMax = 1000
marker_size  = 20
calib_path  = "/home/borg/catkin_ws/src/robot_proj_-frioecalculistabrabo_5a/"
camera_matrix   = np.loadtxt(calib_path+'cameraMatrix_raspi.txt', delimiter=',')
camera_distortion   = np.loadtxt(calib_path+'cameraDistortion_raspi.txt', delimiter=',')
font = cv2.FONT_HERSHEY_PLAIN
#import do aruco 
import cv2.aruco as aruco

#Caso os arucos não estejam lendo bem usar a função abaixo
def valida_aruco(target):
    if ids == target:
        return True
    else:
        return False

#Para usar no loop do robo usar assim:
if valida_aruco(50):
    OK50 = True
    #Demais flags para outros arucos
#################### FIM ARUCO ########################

#Coisas para usar a garra
#Importes nescessarias
from std_msgs.msg import Float64

#Publisher
    ombro = rospy.Publisher("/joint1_position_controller/command", Float64, queue_size=1)
    garra = rospy.Publisher("/joint2_position_controller/command", Float64, queue_size=1)
#Como usar
    ombro.publish(-1.0) ## para baixo
    garra.publish(0.0)  ## Fechado
    rospy.sleep(3.0)
    ombro.publish(1.5) ## para cima
    garra.publish(-1.0) ## Aberto
    rospy.sleep(3.0)
    ombro.publish(0.0) ## para frente

#Função para achar coefieciente angular da reta formada pelos centros dos contornos
def ajuste_linear_x_fy(mask):
    global coef_angular
    """Recebe uma imagem já limiarizada e faz um ajuste linear
        retorna coeficientes linear e angular da reta
        e equação é da forma
        y = coef_angular*x + coef_linear
    """
    pontos = np.where(mask==255)
    ximg = pontos[1]
    yimg = pontos[0] 
    if len(yimg) != 0:
        yimg_c = sm.add_constant(yimg)
        model = sm.OLS(ximg,yimg_c)
        results = model.fit()
        coef_angular = results.params[1] # Pegamos o beta 1
        coef_linear =  results.params[0] # Pegamso o beta 0
        return coef_angular, coef_linear
    else:
        return 0,0

################ TRATA IMAGEM ##########################
#Segmenta cor amarela
def segmenta_linha_amarela(bgr):
    """Não mude ou renomeie esta função
        deve receber uma imagem bgr e retornar os segmentos amarelos do centro da pista em branco.
        Utiliza a função cv2.morphologyEx() para limpar ruidos na imagem
    """
    img_hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(img_hsv, (30, 55, 42), (32, 255, 255))
    final_mask = cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((10, 10)))
    final_mask=morpho_limpa(final_mask)

    return final_mask
#Limpa imagem
def morpho_limpa(mask):
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(12,12))
    mask = cv2.morphologyEx( mask, cv2.MORPH_OPEN, kernel )
    mask = cv2.morphologyEx( mask, cv2.MORPH_CLOSE, kernel )

    return mask

############ REGRESSÂO PELO CENTRO DOS CONTORNOS #################
def ajuste_linear_x_fy(mask):
    global coef_angular
    """Recebe uma imagem já limiarizada e faz um ajuste linear
        retorna coeficientes linear e angular da reta
        e equação é da forma
        y = coef_angular*x + coef_linear
    """
    pontos = np.where(mask==255)
    ximg = pontos[1]
    yimg = pontos[0] 
    if len(yimg) != 0:
        yimg_c = sm.add_constant(yimg)
        model = sm.OLS(ximg,yimg_c)
        results = model.fit()
        coef_angular = results.params[1] # Pegamos o beta 1
        coef_linear =  results.params[0] # Pegamso o beta 0
        return coef_angular, coef_linear
    else:
        return 0,0

#Desenha a linha na mascara
def ajuste_linear_grafico_x_fy(mask):
    """Faz um ajuste linear e devolve uma imagem rgb com aquele ajuste desenhado sobre uma imagem"""
    coef_angular, coef_linear = ajuste_linear_x_fy(mask)
    print("x = {:3f}*y + {:3f}".format(coef_angular, coef_linear))
    pontos = np.where(mask==255) # esta linha é pesada e ficou redundante
    ximg = pontos[1]
    yimg = pontos[0]
    if len(yimg) != 0:
        y_bounds = np.array([min(yimg), max(yimg)])
        x_bounds = coef_angular*y_bounds + coef_linear
        print("x bounds", x_bounds)
        print("y bounds", y_bounds)
        x_int = x_bounds.astype(dtype=np.int64)
        y_int = y_bounds.astype(dtype=np.int64)
        mask_rgb =  cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
        cv2.line(mask_rgb, (x_int[0], y_int[0]), (x_int[1], y_int[1]), color=(0,0,255), thickness=11);    
        return mask_rgb
    else:
        return None
######## ROSTIME ##########
#Condição para parar de girar
w_vel = velocidade angular
start_time = rospy.Time.now()#marca o tempo no instante
if rospy.Time.now() - start_time >= rospy.Duration.from_sec(2.1 * math.pi / w_vel):
########## ODOMETRIA ############
def recebe_odometria(data):
    global x
    global y
    global contador
    global angulos
    global angulo_robo

    x = data.pose.pose.position.x
    y = data.pose.pose.position.y

    quat = data.pose.pose.orientation
    lista = [quat.x, quat.y, quat.z, quat.w]
    angulos = np.degrees(transformations.euler_from_quaternion(lista))  

    angulo_robo = angulos[2]
    angulo_robo = (angulo_robo + 360)%360
#importes
from nav_msgs.msg import Odometry

#Subscriber
recebe_odom = rospy.Subscriber("/odom", Odometry , recebe_odometria)


#Estima ponto de fuga de duas retas
def estimar_linha_nas_faixas(img, mask):
    """Não mude ou renomeie esta função
        deve receber uma imagem preta e branca e retorna dois pontos que formen APENAS uma linha em cada faixa. Desenhe cada uma dessas linhas na iamgem.
         formato: [[(x1,y1),(x2,y2)], [(x1,y1),(x2,y2)]]
    """
    linesP = cv2.HoughLinesP(mask, 1, np.pi / 180, 50, None, 50, 10)

    linhas = []
    esquerda_existe = False


    if linesP is not None:
        for i in range(0, len(linesP)):
            l = linesP[i][0]
            m = (l[3]-l[1])/(l[2]-l[0])
            if m <= 0 and not esquerda_existe:
                cv2.line(img, (l[0], l[1]), (l[2], l[3]), (255,0,255), 2)
                linha = []
                linha.append((l[0],l[1]))
                linha.append((l[2],l[3]))
                linhas.append(linha)
                esquerda_existe = True
            elif m > 0:
                cv2.line(img, (l[0], l[1]), (l[2], l[3]), (255,0,255), 2)
                linha = []
                linha.append((l[0],l[1]))
                linha.append((l[2],l[3]))
                linhas.append(linha)
                print(linhas)
                return linhas
    return None

def calcular_equacao_das_retas(linhas):
    """Não mude ou renomeie esta função
        deve receber dois pontos que estejam em cada uma das faixas e retornar a equacao das duas retas. Onde y = h + m * x. Formato: [(m1,h1), (m2,h2)]
    """
    equacao = []

    for linha in linhas:
        x,y = linha[0]
        x2,y2 = linha[1]

        m = (y-y2)/(x-x2)
        h = y2 - m*x2

        equacao.append((m,h))
    return equacao

def calcular_ponto_de_fuga(img, equacoes):
    """Não mude ou renomeie esta função
        deve receber duas equacoes de retas e retornar o ponto de encontro entre elas. Desenhe esse ponto na imagem.
    """
    m1,h1 = equacoes[0]
    m2,h2 = equacoes[1]

    x = int((h2-h1)/(m1-m2))
    y = int(m2*x + h2)

    img = cv2.circle(img, (x,y), radius=12, color=(255, 0, 255), thickness=-1)

    return img, (x,y)

####### METODOS PARA OPENCV #########
def encontrar_centro_dos_contornos(img, contornos):
    """Não mude ou renomeie esta função
        deve receber um contorno e retornar, respectivamente, a imagem com uma cruz no centro de cada segmento e o centro dele. formato: img, x, y
    """
    img_copia = img.copy()
    X = []
    Y = []
    area = []
    for contorno in contornos:
        M = cv2.moments(contorno)

        # Usando a expressão do centróide
        if M["m00"] != 0: 
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            centro = (cX, cY)
            crosshair(img_copia, centro, 5, [0, 0, 255])
            X.append(cX)
            Y.append(cY)
            area.append(cv2.contourArea(contorno))

    return img_copia, X, Y, area

    def encontrar_contornos(mask):
    """Não mude ou renomeie esta função
        deve receber uma imagem preta e branca os contornos encontrados
    """
    # RETR_EXTERNAL: Apenas Contornos Externos
    contornos, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    return contornos

def crosshair(img, point, size, color):
    """ Desenha um crosshair centrado no point.
        point deve ser uma tupla (x,y)
        color é uma tupla R,G,B uint8
    """
    x,y = point
    cv2.line(img,(x - size,y),(x + size,y),color,2)
    cv2.line(img,(x,y - size),(x, y + size),color,2)

def escrever_textos(frame,box, color, text):
    """
    Escrever os pontos P1 e P2, a bandeira e desenhar um retangulo envolta da bandeira.
    """
    (x,y,w,h) = box
    
    # Retangulo
    cv2.rectangle(frame, (x,y), (x+w,y+h), color, 3)

    font = cv2.FONT_HERSHEY_SIMPLEX
    # Flag
    cv2.putText(frame, text, (x,y+5*h//4), font, 0.75, color, 2, cv2.LINE_AA)
    # P1
    cv2.putText(frame, 'P1 (%s,%s)'%(x,y), (x,y-5), font, 0.5, (255,255,255), 1, cv2.LINE_AA)
    # P2
    cv2.putText(frame, 'P2 (%s,%s)'%(x+w,y+h), (x+w+5,y+h), font, 0.5, (255,255,255), 1, cv2.LINE_AA)
    

    return frame, ((x,y),(x+w,y+h))
