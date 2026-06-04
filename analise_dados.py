import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os

# Configurações visuais para padrão de artigos científicos (IEEE)
plt.rcParams['font.family'] = 'serif'
plt.rcParams['axes.labelsize'] = 10
plt.rcParams['xtick.labelsize'] = 8
plt.rcParams['ytick.labelsize'] = 8
plt.rcParams['legend.fontsize'] = 8
plt.rcParams['axes.titlesize'] = 11

ARQUIVO_CSV = './archives/dados_controle_tello.csv'

def main():
    if not os.path.exists(ARQUIVO_CSV):
        print(f"Erro: Arquivo '{ARQUIVO_CSV}' não encontrado.")
        print("Certifique-se de ter voado com o drone e gerado os dados primeiro.")
        return

    print(f"Lendo dados de {ARQUIVO_CSV}...\n")
    df = pd.read_csv(ARQUIVO_CSV)
    
    tempo = df['Tempo(s)']

    print("="*40)
    print(" ANÁLISE DE DESEMPENHO (MÉTRICAS)")
    print("="*40)
    
    # MAE (Mean Absolute Error) - Mede o quão distante o drone ficou do alvo em média
    mae_x = df['Erro_X(px)'].abs().mean()
    mae_y = df['Erro_Y(px)'].abs().mean()
    mae_z = df['Erro_Z(px)'].abs().mean()

    # Erro Máximo (Overshoot)
    max_x = df['Erro_X(px)'].abs().max()
    max_y = df['Erro_Y(px)'].abs().max()
    max_z = df['Erro_Z(px)'].abs().max()

    print(f"Eixo X (Yaw - Rotação):")
    print(f"  - Erro Médio Absoluto (MAE): {mae_x:.2f} px")
    print(f"  - Erro Máximo: {max_x} px\n")

    print(f"Eixo Y (Altitude):")
    print(f"  - Erro Médio Absoluto (MAE): {mae_y:.2f} px")
    print(f"  - Erro Máximo: {max_y} px\n")

    print(f"Eixo Z (Distância):")
    print(f"  - Erro Médio Absoluto (MAE): {mae_z:.2f} px")
    print(f"  - Erro Máximo: {max_z} px")
    print("="*40)

    # Geração do Gráfico 1: Erros de Posição
    fig, axs = plt.subplots(3, 1, figsize=(7, 6), sharex=True)
    
    # Eixo X
    axs[0].plot(tempo, df['Erro_X(px)'], color='black', linewidth=1.2, label='Erro X (px)')
    axs[0].axhline(0, color='red', linestyle='--', linewidth=1, label='Setpoint (0)')
    axs[0].set_ylabel('Erro Yaw')
    axs[0].set_title('Resposta de Rastreamento (Erros de Posição)')
    axs[0].legend(loc='upper right')
    axs[0].grid(True, linestyle=':', alpha=0.7)

    # Eixo Y
    axs[1].plot(tempo, df['Erro_Y(px)'], color='black', linewidth=1.2, label='Erro Y (px)')
    axs[1].axhline(0, color='red', linestyle='--', linewidth=1, label='Setpoint (0)')
    axs[1].set_ylabel('Erro Altitude')
    axs[1].legend(loc='upper right')
    axs[1].grid(True, linestyle=':', alpha=0.7)

    # Eixo Z
    axs[2].plot(tempo, df['Erro_Z(px)'], color='black', linewidth=1.2, label='Erro Z (px)')
    axs[2].axhline(0, color='red', linestyle=':', linewidth=1)
    axs[2].axhline(30, color='gray', linestyle='--', linewidth=1, label='Zona Morta (+30px)')
    axs[2].axhline(-30, color='gray', linestyle='--', linewidth=1)
    axs[2].set_ylabel('Erro Distância')
    axs[2].set_xlabel('Tempo (s)')
    axs[2].legend(loc='upper right')
    axs[2].grid(True, linestyle=':', alpha=0.7)

    plt.tight_layout()
    plt.savefig('./archives/grafico_erros_ieee.png', dpi=300)
    print("\nGráfico de Erros salvo como: 'grafico_erros_ieee.png'")

    # Geração do Gráfico 2: Sinais de Controle (Esforço de Controle / Saturação)
    fig2, axs2 = plt.subplots(3, 1, figsize=(7, 6), sharex=True)
    
    # Sinal Yaw
    axs2[0].plot(tempo, df['Sinal_Yaw'], color='blue', linewidth=1, label='Sinal Yaw')
    axs2[0].axhline(40, color='red', linestyle=':', linewidth=1, label='Saturação (Max Vel)')
    axs2[0].axhline(-40, color='red', linestyle=':', linewidth=1)
    axs2[0].set_ylabel('Vel. Yaw')
    axs2[0].set_title('Esforço de Controle (Sinais Enviados ao Drone)')
    axs2[0].legend(loc='upper right')
    axs2[0].grid(True, linestyle=':', alpha=0.7)

    # Sinal Altitude
    axs2[1].plot(tempo, df['Sinal_Alt'], color='blue', linewidth=1, label='Sinal Altitude')
    axs2[1].axhline(40, color='red', linestyle=':', linewidth=1)
    axs2[1].axhline(-40, color='red', linestyle=':', linewidth=1)
    axs2[1].set_ylabel('Vel. Altitude')
    axs2[1].legend(loc='upper right')
    axs2[1].grid(True, linestyle=':', alpha=0.7)

    # Sinal Distância
    axs2[2].plot(tempo, df['Sinal_Dist'], color='blue', linewidth=1, label='Sinal Distância')
    axs2[2].axhline(40, color='red', linestyle=':', linewidth=1)
    axs2[2].axhline(-40, color='red', linestyle=':', linewidth=1)
    axs2[2].set_ylabel('Vel. Distância')
    axs2[2].set_xlabel('Tempo (s)')
    axs2[2].legend(loc='upper right')
    axs2[2].grid(True, linestyle=':', alpha=0.7)

    plt.tight_layout()
    plt.savefig('./archives/grafico_sinais_ieee.png', dpi=300)
    print("Gráfico de Sinais salvo como: 'grafico_sinais_ieee.png'")

    # Mostra os gráficos na tela (opcional, pode fechar a janela quando quiser)
    plt.show()

if __name__ == '__main__':
    main()