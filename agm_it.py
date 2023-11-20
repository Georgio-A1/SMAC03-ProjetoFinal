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
# Standard imports
import math
import random

# Third-party imports
import matplotlib.pyplot as plt


class Grafo:
    """
    Representa um grafo, utilizado para simular a rede de sensores sem fio.

    Atributos:
        V (int): Número de vértices no grafo.
        arestas (list): Lista de arestas do grafo.
        coordenadas (dict): Dicionário que mapeia cada vértice às suas
        coordenadas.
        torre (int): Índice do vértice que representa a torre na simulação.
        pesos (dict): Dicionário que mapeia cada vértice ao seu peso
        (energia restante).
    """
    def __init__(self, vertices):
        self.V = vertices
        self.arestas = []
        self.coordenadas = {}
        self.torre = 0
        self.pesos = dict()
        # self.pesos = {i: 100 for i in range(self.V)}
        self.inicializar_pesos()

    def inicializar_pesos(self) -> None:
        """
        Inicializa os pesos (energias) de cada vértice.
        """
        for i in range(self.V):
            self.pesos[i] = 100

    def definir_peso(self, u: int, peso: float) -> None:
        """
        Define o peso (energia restante) de um vértice.

        Parâmetros:
            u (int): Índice do vértice.
            peso (float): Peso (energia) a ser atribuído ao vértice.
        """
        if u in self.pesos:
            self.pesos[u] = peso

    def adicionar_aresta(self, u: int, v: int, w: float) -> None:
        """
        Adiciona uma aresta ao grafo.

        Parâmetros:
            u (int): Índice do primeiro vértice da aresta.
            v (int): Índice do segundo vértice da aresta.
            w (float): Peso (custo) da aresta.
        """
        self.arestas.append([u, v, w])

    def adicionar_coordenada(self, u: int, x: float, y: float) -> None:
        """
        Adiciona coordenadas a um vértice.

        Parâmetros:
            u (int): Índice do vértice.
            x (float): Coordenada x do vértice.
            y (float): Coordenada y do vértice.
        """
        self.coordenadas[u] = (x, y)

    def kruskal(self) -> list:
        """
        Implementa o algoritmo de Kruskal para encontrar a árvore geradora
        mínima (MST) do grafo.

        Retorna:
            list: Lista de arestas que compõem a MST.
        """
        resultado = []
        self.arestas.sort(key=lambda aresta: aresta[2])
        pai = list(range(self.V))

        def encontrar(pai, i):
            if pai[i] == i:
                return i
            return encontrar(pai, pai[i])

        def unir(pai, u, v):
            raiz_u = encontrar(pai, u)
            raiz_v = encontrar(pai, v)
            pai[raiz_u] = raiz_v

        for u, v, w in self.arestas:
            raiz_u = encontrar(pai, u)
            raiz_v = encontrar(pai, v)
            if raiz_u != raiz_v:
                resultado.append([u, v, w])
                unir(pai, raiz_u, raiz_v)

        # Filtrar apenas as arestas que têm caminho até a torre
        caminho_torre = set()
        for i in range(self.V):
            if encontrar(pai, i) == encontrar(pai, self.torre):
                caminho_torre.add(i)

        resultado_filtrado = [
            aresta for aresta in resultado
            if aresta[0] in caminho_torre or aresta[1] in caminho_torre
        ]
        return resultado_filtrado

    def calculate_cost(self, u: int, v: int) -> float:
        """
        Calcula o custo de uma aresta no grafo.

        Parâmetros:
            u (int): Índice do primeiro vértice da aresta.
            v (int): Índice do segundo vértice da aresta.

        Retorna:
            float: Custo calculado da aresta.
        """
        E_ELEC = 50e-9
        EPSILON_AMP = 100e-12
        N = 2
        K = 1000
        TX_POWER = 14
        GATEWAY_RADIO = 100
        fator_adicional = (10**(TX_POWER/10)) * GATEWAY_RADIO

        coord1 = self.coordenadas[u]
        coord2 = self.coordenadas[v]
        distancia = self.distance(coord1, coord2)
        total_cost = \
            E_ELEC * K + EPSILON_AMP * K * distancia**N * fator_adicional
        return total_cost

    def distance(self, coord1: tuple, coord2: tuple) -> float:
        """
        Calcula a distância euclidiana entre dois pontos.

        Parâmetros:
            coord1 (tuple): Coordenadas do primeiro ponto.
            coord2 (tuple): Coordenadas do segundo ponto.

        Retorna:
            float: Distância euclidiana entre os dois pontos.
        """

        x1, y1 = coord1
        x2, y2 = coord2
        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

    def imprimir_resumo_energias(self) -> None:
        """
        Imprime um resumo das energias restantes de cada nó do grafo.
        """
        print("Resumo das Energias dos Nós:")
        for no, peso in self.pesos.items():
            if peso != 100:  # Ignorar nós com energia 100
                print(f"Nó {no}: Energia Restante = {peso:.2f} unidades")


def plotar_grafo(the_grafo: Grafo, mst: list, id_mensagem: int = 0):
    """
    Plota o grafo e a árvore geradora mínima.

    Parâmetros:
        grafo (Grafo): O objeto Grafo a ser plotado.
        arvore_minima (list): Lista de arestas que compõem a
        árvore geradora mínima.
        id_mensagem (int): Identificador da mensagem para a plotagem.
    """
    print("Plotando o grafo com o id da mensagem:", id_mensagem)
    _, ax = plt.subplots(figsize=(8, 8))

    # Adicionando default para evitar divisão por zero
    max_energy = max(the_grafo.pesos.values(), default=100)
    colors = {
        u: plt.cm.RdYlGn(
            the_grafo.pesos[u] / max_energy if max_energy > 0 else 0
            ) for u in the_grafo.pesos
        }

    for u, v, _ in mst:
        x1, y1 = the_grafo.coordenadas[u]
        x2, y2 = the_grafo.coordenadas[v]
        ax.plot([x1, x2], [y1, y2], 'b-')  # Linhas azuis para as arestas

    for u, (x, y) in the_grafo.coordenadas.items():
        # Verificar se o nó ainda existe no dicionário de pesos
        if u in colors:
            color = colors[u]
            if u == the_grafo.torre:
                # Triângulo verde para a torre
                ax.plot(x, y, 'g^', markersize=10)
            else:
                # Nós coloridos baseados na energia
                ax.plot(x, y, 'o', color=color)

    ax.set_title("Árvore Geradora Mínima")
    ax.set_xlabel("Coordenada X")
    ax.set_ylabel("Coordenada Y")
    ax.grid(True)

    # Adicionando a legenda de cores
    sm = plt.cm.ScalarMappable(
        cmap=plt.cm.RdYlGn,
        norm=plt.Normalize(vmin=0, vmax=max_energy)
    )
    sm.set_array([])
    plt.colorbar(sm, label='Energia dos motes', ax=ax, shrink=0.8)

    plt.savefig(f'a{id_mensagem}.png')


def ler_grafo(nome_arquivo) -> Grafo:
    """
    Lê um grafo a partir de um arquivo.

    Parâmetros:
        nome_arquivo (str): Nome do arquivo que contém os dados do grafo.

    Retorna:
        Grafo: Objeto Grafo carregado a partir do arquivo.
    """
    with open(nome_arquivo, 'r', encoding='utf-8') as arquivo:
        linhas = arquivo.readlines()
        quantidade_motes = int(linhas[0])
        print(f"Lendo o grafo com {quantidade_motes} vértices...")
        grapho = Grafo(quantidade_motes)

        indice = 0  # Inicie um índice para os vértices
        for linha in linhas[2:]:
            numeros = linha.split()
            numeros[0] = numeros[0][:-1]
            x, y = map(float, numeros)
            grapho.adicionar_coordenada(indice, x, y)
            # Conecta o novo vértice a todos os anteriores
            for u in range(indice):
                distancia = grapho.distance(grapho.coordenadas[u], (x, y))
                if distancia < 100:
                    grapho.adicionar_aresta(u, indice, distancia)
                    grapho.adicionar_aresta(indice, u, distancia)
            indice += 1

    return grapho


def simular(grapho: Grafo):
    """
    Executa a simulação da rede de sensores sem fio.

    Parâmetros:
        grafo (Grafo): O grafo representando a rede de sensores.
    """
    def buscar_caminho_mst(
            mst: list, atual: int, destino: int, visitados: set) -> list:
        """
        Busca um caminho entre dois nós na MST.

        Parâmetros:
            mst (list): Lista de arestas que compõem a MST.
            atual (int): Índice do nó atual.
            destino (int): Índice do nó destino.
            visitados (set): Conjunto de nós já visitados.

        Retorna:
            list: Lista de nós que compõem o caminho.
        """
        if atual == destino:
            return [destino]
        visitados.add(atual)
        for u, v, _ in mst:  # Alteração aqui para incluir o peso da aresta
            if u == atual and v not in visitados:
                caminho = buscar_caminho_mst(mst, v, destino, visitados)
                if caminho:
                    return [atual] + caminho
            elif v == atual and u not in visitados:
                caminho = buscar_caminho_mst(mst, u, destino, visitados)
                if caminho:
                    return [atual] + caminho
        return []

    def reduzir_peso_caminho(graph: Grafo, caminho: list) -> bool:
        """
        Reduz o peso de um caminho no grafo.

        Parâmetros:
            graph (grafo): O grafo a ser modificado.
            caminho (list): O caminho a ser modificado.

        Retorna:
            bool: Indica se algum nó foi removido.
        """
        nos_removidos = set()
        for i in range(len(caminho) - 1):
            u = caminho[i]
            v = caminho[i + 1]

            # verifica se e torre
            if u == graph.torre or v == graph.torre:
                continue

            custo = graph.calculate_cost(u, v)

            if u in graph.pesos:
                graph.pesos[u] -= custo
                if graph.pesos[u] <= 0:
                    nos_removidos.add(u)

            if v in graph.pesos:
                graph.pesos[v] -= custo
                if graph.pesos[v] <= 0:
                    nos_removidos.add(v)

        for no in nos_removidos:
            del graph.pesos[no]
            mst_nos.discard(no)

        return len(nos_removidos) > 0

    mst_arestas = grapho.kruskal()
    # Atualize para usar as chaves do dicionário de pesos
    mst_nos = set(grapho.pesos.keys())
    print("Iniciando a simulação...")
    i = 1

    while any(grapho.pesos[u] > 0 for u in mst_nos):
        if len(mst_nos) == 1:
            print("Apenas a torre sobrou. Finalizando a simulação.")
            grapho.imprimir_resumo_energias()
            break
        if not mst_arestas:
            print("Não é possível formar uma MST. Finalizando a simulação.")
            grapho.imprimir_resumo_energias()
            break
        if len(mst_nos) == 2:
            print("Apenas a torre e um nó sobraram. Finalizando a simulação.")
            grapho.imprimir_resumo_energias()

            for no in mst_nos:
                if no != grapho.torre:
                    del grapho.pesos[no]
            plotar_grafo(grapho, mst_arestas, id_mensagem=i)
            break
        no_aleatorio = random.choice([
            u for u in mst_nos if grapho.pesos[u] > 0 and u != grapho.torre])
        print(f"Nó aleatório escolhido: {no_aleatorio}")
        caminho = buscar_caminho_mst(
            mst_arestas, no_aleatorio, grapho.torre, set())
        if reduzir_peso_caminho(grapho, caminho):
            print("Recalculando a MST após mudança de energia...")
            # Devemos recalcular a o Kruskal
            # apenas para as arestas que ainda existem
            grapho.arestas = [
                aresta for aresta in grapho.arestas
                if aresta[0] in mst_nos and aresta[1] in mst_nos
            ]
            mst_arestas = grapho.kruskal()

            print("MST recalculada:")
            print(mst_arestas)
            # Atualize o conjunto de nós após a recalculação da MST
            mst_nos = set()
            for u, v, _ in mst_arestas:
                mst_nos.add(u)
                mst_nos.add(v)
            grapho.imprimir_resumo_energias()
            plotar_grafo(
                grapho,
                mst_arestas,
                # id_mensagem=f"{i}_recalculada"
                id_mensagem=i
            )
            if not mst_arestas:
                print("Não é possível formar uma MST. \
                      Finalizando a simulação.")
                grapho.imprimir_resumo_energias()
                break
            i = 1 + i
            plotar_grafo(grapho, mst_arestas, id_mensagem=i)
            grapho.imprimir_resumo_energias()
        else:
            print("Nenhum nó foi removido. Nada a fazer.")
            print("MST atual:", mst_arestas)
            print("Nós com energia:", mst_nos)
            i = 1 + i

    print("Simulação concluída.")


if __name__ == '__main__':
    NOME_ARQUIVO = 'rssf.txt'
    grafo = ler_grafo(NOME_ARQUIVO)
    arvore_minima = grafo.kruskal()
    plotar_grafo(grafo, arvore_minima, id_mensagem=0)
    simular(grafo)
