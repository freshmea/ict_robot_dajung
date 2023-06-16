def test():
    print('A')
    yield 1
    print('B')
    yield 2
    print('C')
    yield 3

def main():
    output = test()
    # print('D')
    # a = next(output)
    # print(a)
    
    # print('E')
    # b = next(output)
    # print(b)
    
    # c = next(output)
    # print(c)
    
    for i in output:
        print(i)

if __name__ == '__main__':
    main()
