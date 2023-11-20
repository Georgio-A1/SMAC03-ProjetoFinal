"""
    Implementação do algoritmo de simulação de energia em redes de sensores
    sem fio

    Autores:    2022003479 - Georgio Georges Aoun
                2021020134 - Gustavo Daniel Vitor
                2020030584 - Luiz Raul Gomes Oliveira
                2021015813 - Matheus Henrique Souza Araujo

    Disciplina: Algoritimos em Grafos
    Professor: Rafael Frinhani

    Universidade Federal de Itajubá - UNIFEI
"""

import networkx as nx
import math
import matplotlib.pyplot as plt
import os

def ler_informacoes_motes(nome_arquivo):
    motes = {}
    with open(nome_arquivo, 'r') as arquivo:
        linhas = arquivo.readlines()
        num_motes = int(linhas[0].strip())
        erbx, erby = map(float, linhas[1].strip().split(', '))
        motes['ERB'] = (erbx, erby)
        for i in range(2, num_motes + 2):
            x, y = map(float, linhas[i].strip().split(', '))
            motes[i - 2] = (x, y)
    return num_motes, motes

def calcular_distancia(coord1, coord2):
    x1, y1 = coord1
    x2, y2 = coord2
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

def calcular_consumo_energia(distancia):
    E_elec = 50e-9  # 50 nJ/bit
    epsilon_amp = 10e-12  # 10 pJ/bit/m^2
    n = 2  # Path loss exponent
    k = 1000  # Number of bits
    tx_power = 14  # Transmission power in dBm
    gateway_ratio = 100  # Sensors to Mesh Gateway ratio

    fator_adicional = (10 ** (tx_power / 10)) * gateway_ratio
    consumo = E_elec * k + epsilon_amp * k * distancia ** n * fator_adicional
    return consumo

# Inicialização do grafo
nome_arquivo = "Cenario 5 - Rede 400.txt"
num_motes, motes = ler_informacoes_motes(nome_arquivo)

G = nx.Graph()
for mote, coords in motes.items():
    G.add_node(mote, pos=coords, energia=100.0)
G.add_node('ERB', pos=motes['ERB'], energia=100.0)

raio_alcance = 100
for i in motes:
    if i != 'ERB':
        for j in motes:
            if j != i and j != 'ERB':
                distance = calcular_distancia(motes[i], motes[j])
                if distance <= raio_alcance:
                    G.add_edge(i, j, weight=distance)
        distance_to_erb = calcular_distancia(motes[i], motes['ERB'])
        if distance_to_erb <= raio_alcance:
            G.add_edge(i, 'ERB', weight=distance_to_erb)

# Construção inicial da AGM
agm = nx.minimum_spanning_tree(G, weight='weight')

# Simulação
num_simulacoes = 2000
pasta_imagens = r"/home/raul/Dropbox/UNIFEI/SMAC03 GRAFOS/Projeto/SMAC03-ProjetoFinal-main/GIf AGM_Simples1"

if not os.path.exists(pasta_imagens):
    os.makedirs(pasta_imagens)

for simulacao in range(num_simulacoes):
    nos_para_remover = set()

    for u, v, data in agm.edges(data=True):
        no_alvo = u if G.nodes[u]['energia'] < G.nodes[v]['energia'] else v
        distancia = data['weight']
        consumo = calcular_consumo_energia(distancia)
        G.nodes[no_alvo]['energia'] -= consumo
        if G.nodes[no_alvo]['energia'] <= 0:
            nos_para_remover.add(no_alvo)

    for no in nos_para_remover:
        if no in G:
            G.remove_node(no)
            if no in agm:  # Remove o nó também da AGM
                agm.remove_node(no)

    if simulacao % 25 == 0:
        plt.figure()
        cores = [G.nodes[node]['energia'] for node in G.nodes()]
        pos = nx.get_node_attributes(agm, 'pos')  # Usando a AGM para plotagem
        nx.draw(agm, pos, with_labels=True, node_color=cores, cmap=plt.cm.RdYlGn, vmin=0.0, vmax=100.0, node_size=50, font_size=8)
        sm = plt.cm.ScalarMappable(cmap=plt.cm.RdYlGn, norm=plt.Normalize(vmin=0.0, vmax=100.0))
        sm.set_array([])
        plt.colorbar(sm, label='Energia dos motes', ax=plt.gca(), shrink=0.8)
        plt.suptitle(f'Simulação no tempo {simulacao}')
        plt.savefig(os.path.join(pasta_imagens, f"simulacao_{simulacao}.png"))
        plt.close()

print('Energia restante dos motes:')
for mote in G.nodes():
    print(f'Mote {mote}: {G.nodes[mote]["energia"]}')
