from djitellopy import Tello
import cv2
import pygame
import numpy as np
import time
import csv

# Configurações gerais de limite e operação do drone
VELOCIDADE_MANUAL = 60
QUADROS_POR_SEGUNDO = 30
VELOCIDADE_MAXIMA = 40

# Ganhos do controlador PD (Proporcional e Derivativo) para cada eixo de movimento
GANHO_YAW = 0.4
GANHO_YAWD = 0.05 

GANHO_ALTITUDE = 0.2
GANHO_ALTITUDED = 0.05 

GANHO_DISTANCIA = 0.4
GANHO_DISTANCIAD = 0.05 

# Parâmetros da câmera e referências do setpoint (alvo)
LARGURA_FRAME = 960
ALTURA_FRAME = 720
LARGURA_ALVO_ROSTO = 180
ZONA_MORTA_DISTANCIA = 30

# Inicialização do classificador visual pré-treinado do OpenCV
detector_rostos = cv2.CascadeClassifier(
    cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
)

# Dicionário de memória para reter estados anteriores, essencial para a ação derivativa
estado_controle = {
    'erro_x_anterior': 0,
    'erro_y_anterior': 0,
    'erro_dist_anterior': 0,
    'tempo_anterior': time.time()
}

def detectar_rosto(frame):
    """
    Processa a imagem em escala de cinza e busca características faciais.
    Filtra os resultados para retornar apenas as coordenadas do maior rosto detectado.
    """
    escala_cinza = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    rostos = detector_rostos.detectMultiScale(
        escala_cinza,
        scaleFactor=1.1,
        minNeighbors=5,
        minSize=(60, 60)
    )

    if len(rostos) == 0:
        return None

    rostos = sorted(rostos, key=lambda r: r[2] * r[3], reverse=True)
    x, y, w, h = rostos[0]
    cx = x + w // 2
    cy = y + h // 2
    
    return cx, cy, w, h


def calcular_velocidades(rosto):
    """
    Implementação do Controlador Proporcional-Derivativo (PD) digital.
    Calcula o sinal de controle (velocidade) com base no erro atual de posição
    e na taxa de variação desse erro ao longo do tempo de amostragem.
    """
    global estado_controle
    
    # Reseta o tempo se o alvo for perdido para evitar saltos na derivada ao reencontrar
    if rosto is None:
        estado_controle['tempo_anterior'] = time.time()
        return 0, 0, 0

    cx, cy, largura, altura = rosto
    centro_x = LARGURA_FRAME // 2
    centro_y = ALTURA_FRAME // 2
    
    # Cálculo dos erros atuais de deslocamento em relação ao centro da tela e tamanho alvo
    erro_x = cx - centro_x
    erro_y = cy - centro_y
    erro_distancia = largura - LARGURA_ALVO_ROSTO

    # Tempo de amostragem decorrido desde a última iteração
    tempo_atual = time.time()
    dt = tempo_atual - estado_controle['tempo_anterior']
    if dt <= 0:
        dt = 0.033 

    # Aplicação da zona morta no eixo Z para evitar trepidações quando o drone está próximo da distância ideal
    erro_dist_ativo = 0
    if abs(erro_distancia) >= ZONA_MORTA_DISTANCIA:
        zona_com_sinal = ZONA_MORTA_DISTANCIA if erro_distancia > 0 else -ZONA_MORTA_DISTANCIA
        erro_dist_ativo = erro_distancia - zona_com_sinal

    # Ação derivativa: taxa de variação do erro dividida pelo tempo
    derivada_x = (erro_x - estado_controle['erro_x_anterior']) / dt
    derivada_y = (erro_y - estado_controle['erro_y_anterior']) / dt
    derivada_dist = (erro_dist_ativo - estado_controle['erro_dist_anterior']) / dt

    # Atualização da memória do controlador para o próximo ciclo
    estado_controle['erro_x_anterior'] = erro_x
    estado_controle['erro_y_anterior'] = erro_y
    estado_controle['erro_dist_anterior'] = erro_dist_ativo
    estado_controle['tempo_anterior'] = tempo_atual

    # Cálculo da lei de controle PD final combinando erro e derivada
    novo_yaw = int((GANHO_YAW * erro_x) + (GANHO_YAWD * derivada_x))
    nova_altitude = int((-GANHO_ALTITUDE * erro_y) - (GANHO_ALTITUDED * derivada_y))
    novo_fb = int((-GANHO_DISTANCIA * erro_dist_ativo) - (GANHO_DISTANCIAD * derivada_dist))

    # Saturação do sinal de controle para respeitar os limites mecânicos e de segurança do drone
    novo_yaw      = max(-VELOCIDADE_MAXIMA, min(VELOCIDADE_MAXIMA, novo_yaw))
    nova_altitude = max(-VELOCIDADE_MAXIMA, min(VELOCIDADE_MAXIMA, nova_altitude))
    novo_fb       = max(-VELOCIDADE_MAXIMA, min(VELOCIDADE_MAXIMA, novo_fb))

    return novo_yaw, nova_altitude, novo_fb


class FrontEnd(object):
    """
    Módulo de interface visual e interface homem-máquina.
    Responsável por gerenciar os eventos do Pygame, renderizar o HUD 
    e centralizar o envio de comandos para o hardware do drone.
    """

    def __init__(self):
        pygame.init()
        pygame.display.set_caption("Tello Face Tracking")
        self.screen = pygame.display.set_mode([LARGURA_FRAME, ALTURA_FRAME])

        self.tello = Tello()

        self.vel_frente_tras   = 0
        self.vel_esquerda_dir  = 0
        self.vel_altitude      = 0
        self.vel_yaw           = 0
        self.velocidade_base   = 10 

        self.enviando_rc       = False 
        self.face_tracking     = False 

        # Variáveis dedicadas à coleta de dados em tempo de execução para análises e plots
        self.dados_voo = []          
        self.tempo_inicio_tracking = 0

        pygame.time.set_timer(pygame.USEREVENT + 1, 1000 // QUADROS_POR_SEGUNDO)

    def run(self):
        """Loop principal do sistema, processando telemetria, visão e controle contínuo."""
        self.tello.connect()
        self.tello.set_speed(self.velocidade_base)
        self.tello.streamoff()
        self.tello.streamon()

        leitura_frame = self.tello.get_frame_read()
        deve_parar = False

        while not deve_parar:
            for evento in pygame.event.get():
                if evento.type == pygame.USEREVENT + 1:
                    self.atualizar()
                elif evento.type == pygame.QUIT:
                    deve_parar = True
                elif evento.type == pygame.KEYDOWN:
                    if evento.key == pygame.K_ESCAPE:
                        deve_parar = True
                    else:
                        self.tecla_pressionada(evento.key)
                elif evento.type == pygame.KEYUP:
                    self.tecla_solta(evento.key)

            if leitura_frame.stopped:
                break

            self.screen.fill([0, 0, 0])
            frame = leitura_frame.frame.copy()

            # Chamada principal do rastreamento se os modos autônomos estiverem ativos
            rosto = detectar_rosto(frame)

            if self.face_tracking and self.enviando_rc:
                self.vel_yaw, self.vel_altitude, self.vel_frente_tras = calcular_velocidades(rosto)
                
                # Coleta e registro das métricas do controlador para posterior plotagem e análise
                erro_x = estado_controle['erro_x_anterior']
                erro_y = estado_controle['erro_y_anterior']
                erro_z = estado_controle['erro_dist_anterior']
                tempo_decorrido = time.time() - self.tempo_inicio_tracking
                
                self.dados_voo.append([
                    round(tempo_decorrido, 3), 
                    erro_x, erro_y, erro_z, 
                    self.vel_yaw, self.vel_altitude, self.vel_frente_tras
                ])

            # Renderização gráfica do HUD na tela de visualização
            if rosto is not None:
                cx, cy, w, h = rosto
                x1, y1 = cx - w // 2, cy - h // 2
                erro_distancia = w - LARGURA_ALVO_ROSTO

                # Define as cores indicativas do status da zona morta (distância)
                if abs(erro_distancia) < ZONA_MORTA_DISTANCIA:
                    cor_box   = (0, 255, 0)
                    label_dist = "OK"
                elif erro_distancia > 0:
                    cor_box   = (0, 100, 255)
                    label_dist = "PERTO"
                else:
                    cor_box   = (255, 200, 0)
                    label_dist = "LONGE"

                cv2.rectangle(frame, (x1, y1), (x1 + w, y1 + h), cor_box, 2)
                cv2.circle(frame, (cx, cy), 5, cor_box, -1)
                cv2.line(frame, (LARGURA_FRAME // 2, ALTURA_FRAME // 2), (cx, cy), (255, 165, 0), 1)

                bx, by, bh = LARGURA_FRAME - 30, 60, 200
                fill = int(np.clip((w / (LARGURA_ALVO_ROSTO * 2)) * bh, 0, bh))
                cv2.rectangle(frame, (bx, by), (bx + 18, by + bh), (60, 60, 60), -1)
                cv2.rectangle(frame, (bx, by + bh - fill), (bx + 18, by + bh), cor_box, -1)
                meio_barra = by + bh // 2
                cv2.line(frame, (bx - 4, meio_barra), (bx + 22, meio_barra), (255, 255, 255), 1)
                cv2.putText(frame, "DIST", (bx - 4, by - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
                cv2.putText(frame, label_dist, (bx - 10, by + bh + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, cor_box, 1)

            cx_frame, cy_frame = LARGURA_FRAME // 2, ALTURA_FRAME // 2
            cv2.line(frame, (cx_frame - 20, cy_frame), (cx_frame + 20, cy_frame), (255, 255, 255), 1)
            cv2.line(frame, (cx_frame, cy_frame - 20), (cx_frame, cy_frame + 20), (255, 255, 255), 1)

            cv2.putText(frame, f"Bateria: {self.tello.get_battery()}%", (10, ALTURA_FRAME - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.putText(frame, f"Tracking: {'ON' if self.face_tracking else 'OFF [F]'}", (10, ALTURA_FRAME - 35), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0) if self.face_tracking else (180, 180, 180), 2)
            cv2.putText(frame, f"Rosto: {'Detectado' if rosto else 'Nenhum'}", (10, ALTURA_FRAME - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0) if rosto else (0, 100, 255), 2)

            frame_disp = np.rot90(frame)
            frame_disp = np.flipud(frame_disp)
            superficie = pygame.surfarray.make_surface(frame_disp)
            self.screen.blit(superficie, (0, 0))
            pygame.display.update()

            time.sleep(1 / QUADROS_POR_SEGUNDO)

        # Trata o desligamento do sistema e executa a exportação dos dados coletados
        if len(self.dados_voo) > 0:
            print("\nSalvando log de voo em 'dados_controle_tello.csv'...")
            with open('dados_controle_tello.csv', mode='w', newline='') as arquivo_csv:
                escritor = csv.writer(arquivo_csv)
                escritor.writerow(['Tempo(s)', 'Erro_X(px)', 'Erro_Y(px)', 'Erro_Z(px)', 'Sinal_Yaw', 'Sinal_Alt', 'Sinal_Dist'])
                escritor.writerows(self.dados_voo)
            print("Log salvo com sucesso! Pronto para plotar no artigo.")

        self.tello.end()

    def tecla_pressionada(self, tecla):
        """Gerencia os comandos manuais diretos de movimentação."""
        if tecla == pygame.K_UP: self.vel_frente_tras = VELOCIDADE_MANUAL
        elif tecla == pygame.K_DOWN: self.vel_frente_tras = -VELOCIDADE_MANUAL
        elif tecla == pygame.K_LEFT: self.vel_esquerda_dir = -VELOCIDADE_MANUAL
        elif tecla == pygame.K_RIGHT: self.vel_esquerda_dir = VELOCIDADE_MANUAL
        elif tecla == pygame.K_w: self.vel_altitude = VELOCIDADE_MANUAL
        elif tecla == pygame.K_s: self.vel_altitude = -VELOCIDADE_MANUAL
        elif tecla == pygame.K_a: self.vel_yaw = -VELOCIDADE_MANUAL
        elif tecla == pygame.K_d: self.vel_yaw = VELOCIDADE_MANUAL

    def tecla_solta(self, tecla):
        """Zera os sinais de controle ou atua como gatilho para estados do drone (tracking/decolagem)."""
        if tecla in (pygame.K_UP, pygame.K_DOWN): self.vel_frente_tras = 0
        elif tecla in (pygame.K_LEFT, pygame.K_RIGHT): self.vel_esquerda_dir = 0
        elif tecla in (pygame.K_w, pygame.K_s):
            if not self.face_tracking: self.vel_altitude = 0
        elif tecla in (pygame.K_a, pygame.K_d):
            if not self.face_tracking: self.vel_yaw = 0
        
        elif tecla == pygame.K_t:
            self.tello.takeoff()
            self.enviando_rc = True
        elif tecla == pygame.K_l:
            self.tello.land()
            self.enviando_rc   = False
            self.face_tracking = False
        
        elif tecla == pygame.K_f:
            self.face_tracking = not self.face_tracking
            if self.face_tracking:
                self.tempo_inicio_tracking = time.time()
                print("Gravacao de dados iniciada!")
            else:
                self.vel_yaw         = 0
                self.vel_altitude    = 0
                self.vel_frente_tras = 0
                print("Gravacao de dados pausada.")

    def atualizar(self):
        """Envia os pacotes de comando de rádio controle (RC) via conexão Wi-Fi para o Tello."""
        if self.enviando_rc:
            self.tello.send_rc_control(
                self.vel_esquerda_dir,
                self.vel_frente_tras,
                self.vel_altitude,
                self.vel_yaw,
            )


def main():
    interface = FrontEnd()
    interface.run()


if __name__ == '__main__':
    main()