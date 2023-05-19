#! /usr/bin/env python3

# Implementação de fila usando lista encadeada
# Fila - First-In, First-Out (FIFO)

class EmptyQueueException(Exception):
    pass

class ElementoFila():

    def __init__(self, valor, proximo):
        self.valor = valor
        self.proximo = proximo

class Fila():

    def __init__(self):
        self.primeiro = None
        self.ultimo = None # None porque ainda não tem nada na fila
        self.qtd = 0 # Quantidade de elementos que tem na fila. A princípio, zero


    def push(self, x): # Push serve para adicionar um elemento no final da fila (ou seja, o que está no topo de uma pilha)
        if self.qtd == 0:
            novo_elemento = ElementoFila(valor=x, proximo=None)
            self.ultimo = novo_elemento
            self.primeiro = novo_elemento
        else:
            novo_elemento = ElementoFila(valor=x, proximo=None)
            self.ultimo.proximo = novo_elemento
            self.ultimo = novo_elemento
        self.qtd += 1

    def pop(self): # Pop serve para remover o último elemento adicionado à fila (ou seja, o que está no começo de uma pilha). O ÚLTIMO da fila.
        if self.qtd == 0:
            raise EmptyQueueException("A fila está vazia")
        valor_saida = self.primeiro.valor
        if self.qtd == 1:
            self.primeiro = None
            self.ultimo = None
            self.qtd -= 1
            return valor_saida
        self.primeiro = self.primeiro.proximo
        self.qtd -= 1
        return valor_saida

    def __repr__(self):
        if self.qtd == 0: # Guard clause
            return "Fila vazia =("
        return f"Fila({self.qtd} | {self.primeiro.valor} -> {self.ultimo.valor})" # Retorna a quantidade de elementos na fila e o primeiro e último elemento

def main():
    f = Fila()
    print(f)
    f.push(3)
    print(f)
    f.push(4)
    print(f)
    f.push(5)
    print(f)
    f.pop()
    print(f)
    f.pop()
    print(f)
    f.pop()
    print(f)

if __name__ == '__main__':
    main()