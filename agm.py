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


# Função para ler as informações dos motes da semente
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
    return motes


# Função para calcular a distância euclidiana entre dois pontos
def calcular_distancia(coord1, coord2):
    x1, y1 = coord1
    x2, y2 = coord2
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


# Função para calcular o consumo de energia
def calcular_consumo_energia(distancia):
    E_elec = 50e-9  # 50 nJ/bit, custo energético para transmitir
    epsilon_amp = 10e-12  # 10 pJ/bit/m^2, custo para ampliar o sinal
    n = 2  # expoente de perda de caminho
    k = 1000  # número de bits transmitidos
    tx_power = 14  # poder de transmissão em dBm (do datasheet)
    gateway_ratio = 100  # relação entre sensores e o Mesh Gateway (do datasheet)

    # Fator adicional para levar em conta o poder de transmissão e a relação com o Mesh Gateway
    fator_adicional = (10 ** (tx_power / 10)) * gateway_ratio

    consumo = E_elec * k + epsilon_amp * k * distancia ** n * fator_adicional
    return consumo


# Inicializar o grafo e adicionar nós e arestas
nome_arquivo = "Cenario 5 - Rede 400.txt"
motes = ler_informacoes_motes(nome_arquivo)
G = nx.Graph()
for mote, coords in motes.items():
    G.add_node(mote, pos=coords, energia=100.0)  # Energia inicial de 100 para cada mote

raio_alcance = 100
for i in range(len(motes)):
    for j in range(i + 1, len(motes)):
        node1 = list(motes.keys())[i]
        node2 = list(motes.keys())[j]
        distance = calcular_distancia(motes[node1], motes[node2])
        if distance <= raio_alcance:
            G.add_edge(node1, node2, weight=distance)

# Calcular distâncias até o ERB
distancias_ate_erb = nx.single_source_dijkstra_path_length(G, 'ERB')

# Simulação
num_simulacoes = 2000
simulacoes_para_recorte = []
grafos_recortes = {}
energias_recortes = {simulacao: {mote: G.nodes[mote]['energia'] for mote in G.nodes()} for simulacao in range(num_simulacoes + 1)}

# Caminho para a pasta onde as imagens serão salvas
pasta_imagens = r"/home/raul/Dropbox/UNIFEI/SMAC03 GRAFOS/Projeto/SMAC03-ProjetoFinal-main/Gif AGM 3"

# Criar a pasta se ela não existir
if not os.path.exists(pasta_imagens):
    os.makedirs(pasta_imagens)

# Simulação e plotagem
for simulacao in range(num_simulacoes):
    # Reconstruir a AGM a cada iteração
    agm = nx.minimum_spanning_tree(G, weight='weight')

    # Lista para armazenar motes com energia esgotada
    motes_esgotados = []

    # Atualizar as distâncias antes de percorrer as arestas da AGM
    distancias_ate_erb = nx.single_source_dijkstra_path_length(G, 'ERB')

    for u, v, data in agm.edges(data=True):
        # Verifique se os nós ainda existem no grafo antes de acessar as distâncias
        if u in G.nodes and v in G.nodes and u in distancias_ate_erb and v in distancias_ate_erb:
            no_alvo = u if distancias_ate_erb[u] > distancias_ate_erb[v] else v

            if G.nodes[no_alvo]['energia'] > 0:
                distancia = data['weight']
                consumo = calcular_consumo_energia(distancia)
                energia_restante = G.nodes[no_alvo]['energia'] - consumo
                G.nodes[no_alvo]['energia'] = max(energia_restante, 0)

                if energia_restante <= 0:
                    motes_esgotados.append(no_alvo)

    # Anunciar motes esgotados e removê-los da AGM
    for mote in motes_esgotados:
        print(f"Mote {mote} esgotou sua energia na iteração {simulacao + 1}")
        G.remove_node(mote)

    # Reconstruir a AGM com os nós restantes
    if G.nodes():  # Verifica se ainda há nós no grafo
        agm = nx.minimum_spanning_tree(G, weight='weight')
        distancias_ate_erb = nx.single_source_dijkstra_path_length(G, 'ERB')

    if simulacao + 1 in simulacoes_para_recorte:
        grafos_recortes[simulacao + 1] = agm.copy()
        energias_recortes[simulacao + 1] = {mote: G.nodes[mote]['energia'] for mote in G.nodes()}

    # Definir as plotagens que serão salvas
    if simulacao % 25 == 0:  # Plota a cada 25 simulações
        plt.figure()
        agm = nx.minimum_spanning_tree(G, weight='weight')
        cores = [G.nodes[node]['energia'] for node in agm.nodes()]
        pos = nx.get_node_attributes(agm, 'pos')
        nx.draw(agm, pos, with_labels=True, node_color=cores, cmap=plt.cm.RdYlGn, vmin=0.0, vmax=100.0, node_size=50, font_size=8)
        sm = plt.cm.ScalarMappable(cmap=plt.cm.RdYlGn, norm=plt.Normalize(vmin=0.0, vmax=100.0))
        sm.set_array([])
        plt.colorbar(sm, label='Energia dos motes', ax=plt.gca(), shrink=0.8)
        plt.suptitle(f'Simulação no tempo {simulacao}')

        # Salvar a figura
        plt.savefig(os.path.join(pasta_imagens, f"simulacao_{simulacao}.png"))

        plt.close()

# Exibir a energia restante dos motes
print('Energia restante dos motes:')
for mote, energia in G.nodes(data='energia'):
    print(f'Mote {mote}: {energia}')
