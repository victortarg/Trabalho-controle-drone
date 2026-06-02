# Tello Face Tracking - Controle Digital PD

Este projeto foi desenvolvido como parte da disciplina de **Sistemas de Controle Digitais**. Ele implementa um sistema de rastreamento facial autônomo em um drone DJI Tello, utilizando Visão Computacional (OpenCV) para a medição do erro e um **Controlador Proporcional-Derivativo (PD)** discreto para estabilizar o voo em três eixos (Yaw, Altitude e Distância).

## Funcionalidades

- **Rastreamento Autônomo:** O drone identifica e centraliza um rosto na tela usando o classificador Haar Cascade.
- **Controlador Digital PD:** Cálculo de velocidade em tempo real baseado no erro de posição (em pixels) e na sua taxa de variação ($de/dt$), garantindo um rastreamento suave e mitigando oscilações (overshoot).
- **HUD (Head-Up Display):** Interface construída em Pygame que exibe a transmissão de vídeo ao vivo, nível de bateria, status do tracking e um indicador visual de distância.
- **Data Logging (Caixa Preta):** Exportação automática de métricas de voo para um arquivo `.csv` para análise e plotagem de gráficos de resposta ao degrau (ideal para artigos modelo IEEE).

---

## Pré-requisitos e Instalação

Certifique-se de ter o Python 3.8 ou superior instalado.

1. Clone este repositório para a sua máquina.
2. Abra o terminal na pasta do projeto e instale as dependências executando:

```bash
pip install -r requirements.txt

```

_(Dependências: `djitellopy`, `opencv-python`, `pygame`, `numpy`)_

---

## 🎮 Como Operar o Drone

1. Ligue o seu DJI Tello.
2. Conecte o Wi-Fi do seu computador à rede gerada pelo Tello (ex: `TELLO-XXXXXX`).
3. Execute o script principal:

```bash
python nome_do_seu_arquivo.py

```

4. A janela do Pygame se abrirá com o vídeo da câmera do drone. Utilize os comandos abaixo:

| Tecla               | Ação                                                         |
| ------------------- | ------------------------------------------------------------ |
| **T**               | Decolar (Takeoff)                                            |
| **L**               | Pousar (Land)                                                |
| **F**               | Ativar / Desativar Face Tracking Automático                  |
| **Setas (↑ ↓ ← →)** | Controle Manual: Mover para frente, trás, esquerda e direita |
| **W / S**           | Controle Manual: Subir / Descer (Altitude)                   |
| **A / D**           | Controle Manual: Girar no próprio eixo (Yaw)                 |
| **ESC**             | Pousar com emergência e fechar o programa                    |

> **Atenção:** O rastreamento facial (tecla F) só deve ser ativado após o drone ter decolado (tecla T) e estar a uma altura segura.

---

## Modelagem e Teoria de Controle

A cinemática do drone atua como uma planta integradora (Tipo 1), onde os comandos de entrada são velocidades que se acumulam na posição física do VANT.

O sistema foi modelado para três eixos distintos, utilizando a seguinte lei de controle digital:

$$u(k) = (K_p \cdot e(k)) + (K_d \cdot \frac{e(k) - e(k-1)}{T_s})$$

- **Eixo X (Yaw):** Centraliza o rosto horizontalmente na imagem.
- **Eixo Y (Altitude):** Centraliza o rosto verticalmente na imagem.
- **Eixo Z (Distância - Pitch):** Mantém uma distância segura baseada na largura da _bounding box_ (alvo de 180px). Possui uma **Zona Morta** de tolerância ($\pm 30$ px) para evitar trepidações em regime permanente.

_Os ganhos ($K_p$ e $K_d$) podem ser ajustados nas variáveis globais no topo do código._

---

## Coleta de Dados para o Artigo IEEE

Para gerar os gráficos de análise de estabilidade e regime permanente exigidos pela disciplina:

1. Ative o tracking pressionando **F**. O console avisará que a gravação começou.
2. Realize os testes físicos (ex: dar um passo para gerar uma entrada em degrau).
3. Encerre o voo fechando o programa (ESC).
4. O sistema gerará automaticamente um arquivo chamado `dados_controle_tello.csv` na raiz do projeto, contendo colunas de tempo, erros (X, Y, Z) e os sinais de controle (velocidades) enviados ao drone em cada iteração.
