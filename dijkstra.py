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
import random
import matplotlib.pyplot as plt

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

# Função para remover um nó do grafo
def remover_no(G, no):
    G.remove_node(no)
    print(f"Mote {no} esgotou sua energia na iteração {simulacao + 1}")

def calcular_consumo_energia(distancia):
    E_elec = 50e-9  # 50 nJ/bit, custo energético para transmitir
    epsilon_amp = 10e-12  # 10 pJ/bit/m^2, custo para ampliar o sinal
    n = 2  # expoente de perda de caminho, representa diminuição do sinal com o aumento da distância
    k = 1000  # número de bits transmitidos
    tx_power = 14  # poder de transmissão em dBm (do datasheet)
    gateway_ratio = 100  # relação entre sensores e o Mesh Gateway (do datasheet)

    # Fator adicional para levar em conta o poder de transmissão e a relação com o Mesh Gateway
    fator_adicional = (10**(tx_power/10)) * gateway_ratio

    consumo = E_elec * k + epsilon_amp * k * distancia ** n * fator_adicional
    return consumo

# Função para encontrar o caminho mínimo
def dijkstra(G, origem):
    distancias = {node: float('inf') for node in G.nodes()}
    predecessores = {node: None for node in G.nodes()}
    distancias[origem] = 0

    nao_visitados = set(G.nodes())

    while nao_visitados:
        no_atual = min(nao_visitados, key=lambda node: distancias[node])
        nao_visitados.remove(no_atual)

        for vizinho in G.neighbors(no_atual):
            peso_aresta = G[no_atual][vizinho]['weight']
            distancia_total = distancias[no_atual] + peso_aresta
            if distancia_total < distancias[vizinho]:
                distancias[vizinho] = distancia_total
                predecessores[vizinho] = no_atual

    return distancias, predecessores

# Criar um grafo não direcionado
G = nx.Graph()

# Nome do arquivo com informações dos motes
# Caso não funcionar coloque o caminho completo para o arquivo
nome_arquivo = r"C:\Users\Georgio\Desktop\Cenário 5 - Rede 400.txt"

# Ler as informações dos motes da semente
motes = ler_informacoes_motes(nome_arquivo)

# Adicionar nós (motes) ao grafo com suas coordenadas como atributo
for mote, coords in motes.items():
    G.add_node(mote, pos=coords)

# Calcular as distâncias e atribuir como pesos das arestas
raio_alcance = 100
for i in range(len(motes)):
    for j in range(i + 1, len(motes)):
        node1 = list(motes.keys())[i]
        node2 = list(motes.keys())[j]
        distance = calcular_distancia(motes[node1], motes[node2])
        if distance <= raio_alcance:
            G.add_edge(node1, node2, weight=distance)

# Dicionário para armazenar a energia de cada mote
energia_motes = {mote: 100.0 for mote in G.nodes()}

# Definindo o nó de destino
destino = 'ERB'  # Nó ERB

# Definindo a semente inicial para reprodução
random.seed(42)

# Número de simulações realizadas
num_simulacoes = 1000  # Alterar conforme necessário

# Lista que armazena os valores dos recortes temporais que queremos plotar
simulacoes_para_recorte = [1, 25, 50, 100, 200, 250, 1000]  # Alterar conforme necessário

# Dicionário para armazenar os grafos em cada recorte temporal
grafos_recortes = {}

# Dicionário para armazenar as energias de cada mote em cada recorte temporal
energias_recortes = {simulacao: {mote: energia_motes[mote] for mote in G.nodes()} for simulacao in range(num_simulacoes + 1)}

# Variável para controlar quando criar recortes temporais
criar_recorte_apos = simulacoes_para_recorte.pop(0)

# Criação do primeiro recorte antes do loop principal
grafos_recortes[1] = G.copy()

# Loop principal das simulações
for simulacao in range(num_simulacoes):
    # Origem aleatória
    origem = random.choice(list(G.nodes()))
    # Chamada da função de dijkstra para caminho mínimo
    distancias_minimas, predecessores = dijkstra(G, origem)

    if simulacao + 1 in simulacoes_para_recorte:
        # Registra uma cópia do grafo no recorte temporal
        grafos_recortes[simulacao + 1] = G.copy()
        # Armazena as energias dos motes no recorte temporal
        energias_recortes[simulacao + 1] = {mote: energia_motes[mote] for mote in G.nodes()}
        if simulacoes_para_recorte:
            criar_recorte_apos = simulacoes_para_recorte.pop(0)

    caminho_minimo = [destino]
    consumo_energia_total = 0.0
    caminho_perdendo_energia = [origem]  # Adiciona o primeiro mote à lista

    while destino in G.nodes() and destino != origem:
        vizinho = predecessores[destino]
        peso_aresta = G[vizinho][destino]['weight']

        # Definir energia_perdida
        energia_perdida = calcular_consumo_energia(peso_aresta)

        if destino != 'ERB':
            # Verificar se o nó está no grafo antes de prosseguir
            if destino in G.nodes():
                # Verificar se há energia suficiente antes de prosseguir
                if energia_motes[destino] >= energia_perdida:
                    energia_motes[destino] -= energia_perdida
                    caminho_perdendo_energia.append(destino)
                else:
                    # Se não houver energia suficiente, remova o nó
                    remover_no(G, destino)
                    # Atualizar o destino para um nó vizinho válido
                    vizinhos_destino = list(G.neighbors(vizinho))
                    if vizinhos_destino:
                        destino = random.choice(vizinhos_destino)
                    else:
                        print('Todos os nós morreram. Simulação encerrada.')
                        break
            else:
                # Se o destino não estiver mais no grafo, atualize para um nó vizinho válido
                vizinhos_destino = list(G.neighbors(vizinho))
                if vizinhos_destino:
                    destino = random.choice(vizinhos_destino)
                else:
                    print('Todos os nós morreram. Simulação encerrada.')
                    break

        consumo_energia_total += energia_perdida
        caminho_minimo.insert(0, vizinho)
        destino = vizinho

    # Subtrair o consumo de energia do primeiro mote (origem)
    if origem != 'ERB' and len(caminho_minimo) >= 2:
        vizinhos_origem = list(G.neighbors(origem))
        proximo_no = caminho_minimo[1]

        while proximo_no not in vizinhos_origem:
            proximo_no = predecessores[proximo_no]

        primeira_aresta = G[origem][proximo_no]['weight']
        energia_perdida = calcular_consumo_energia(primeira_aresta)

        # Verificar se há energia suficiente antes de prosseguir
        if energia_motes[origem] >= energia_perdida:
            energia_motes[origem] -= energia_perdida
        else:
            # Se não houver energia suficiente, a energia é definida como zero
            energia_motes[origem] = 0

    # Imprimir o caminho mínimo e os motes que perdem energia
    #print(f'Simulação {simulacao + 1}: Origem {origem} até a ERB: {caminho_minimo}')
    #print(f'Motes perdendo energia: {caminho_perdendo_energia}')
    #print(f'Consumo de energia total: {consumo_energia_total}')
    #print()

    # Verificar se o grafo ficou vazio após a remoção do nó
    if not G.nodes():
        print('Todos os nós morreram. Simulação encerrada.')
        break

# Plotar os grafos em cada recorte temporal
for recorte, grafo_recorte in grafos_recortes.items():
    plt.figure()

    # Cores com base na energia dos motes no recorte temporal
    cores = [energias_recortes[recorte][node] for node in grafo_recorte.nodes()]
    pos = nx.get_node_attributes(grafo_recorte, 'pos')

    # Desenha os vértices com cores proporcionais à energia
    nx.draw(grafo_recorte, pos, with_labels=True, node_color=cores, cmap=plt.cm.RdYlGn, vmin=0.0, vmax=100.0)

    # Criação de um objeto para a barra de cores
    sm = plt.cm.ScalarMappable(cmap=plt.cm.RdYlGn, norm=plt.Normalize(vmin=0.0, vmax=100.0))
    sm.set_array([])  # você precisa definir um array vazio aqui

    # Adiciona a barra de cores
    plt.colorbar(sm, label='Energia dos motes', ax=plt.gca(), shrink=0.8)

    # Adiciona um título com o número da simulação
    plt.suptitle(f'Simulação no tempo {recorte}')
    plt.savefig(f'grafo_recorte_{recorte}.png')
    plt.show()

print('Energia restante dos motes:')
for mote, energia in energia_motes.items():
    print(f'Mote {mote}: {energia}')
