#! /usr/bin/env python3

# "Fila Nutella" - Implementação de uma fila usando deque
# Deque é uma estrutura de dados que permite inserção e remoção de elementos em ambas as extremidades da fila (Double-Ended Queue)
# Queue é uma estrutura de dados que permite a inserção de elementos ao final da fila e a remoção de elementos do início da fila

from collections import deque

def main():
    fila = deque()
    print(fila)
    fila.append(1)
    print(fila)
    fila.append(2)
    print(fila)
    fila.append(3)
    print(fila)
    fila.popleft() # popleft remove o primeiro elemento da fila
    print(fila)
    fila.popleft()
    print(fila)
    fila.popleft()
    print(fila)


if __name__ == "__main__":
    main()