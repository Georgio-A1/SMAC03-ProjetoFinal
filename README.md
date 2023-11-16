# Simulação de Rede de Sensores sem Fio com Consumo de Energia

Este código implementa uma simulação de uma rede de sensores sem fio, onde os sensores (motes) têm limitações de energia. A simulação utiliza o algoritmo de Dijkstra para encontrar o caminho mínimo de um sensor aleatório até um ponto central chamado ERB (Estação Rádio Base). Durante a simulação, os sensores perdem energia ao transmitir dados, e aqueles com energia insuficiente são removidos da rede.

## Implementação

- O código utiliza a biblioteca NetworkX para representar o grafo.
- Cada nó no grafo representa um mote, e as arestas têm pesos correspondentes à distância euclidiana entre os motes.
- A simulação é repetida várias vezes (definido por `num_simulacoes`).
- A cada simulação, um mote aleatório é escolhido como origem, e o caminho mínimo até a ERB é calculado.
- Os motes perdem energia durante a transmissão, e aqueles sem energia suficiente são removidos.
- O consumo total de energia é registrado, e a simulação pode ser interrompida se todos os motes forem removidos.

## Visualização

- O código gera vários grafos de diferentes pontos temporais, destacando a energia dos motes.
- A cor dos nós representa a energia restante, proporcionando uma representação visual do estado da rede em cada ponto temporal.

## Bibliotecas

Este código faz uso das seguintes bibliotecas Python:

- **[networkx](https://networkx.github.io/):** Biblioteca para criação, manipulação e estudo de estruturas, dinâmicas e funções de redes complexas.
- **[matplotlib](https://matplotlib.org/):** Biblioteca para criação de visualizações estáticas, animadas e interativas em Python.
- **[random](https://docs.python.org/3/library/random.html):** Módulo para geração de números pseudoaleatórios. Utilizado para selecionar nós aleatórios durante a simulação.
- **[math](https://docs.python.org/3/library/math.html):** Módulo que fornece funções matemáticas básicas. Utilizado para calcular a distância euclidiana entre coordenadas.
