def main():
    output = [i for i in range(100) if f'{i:b}'.count('0') == 1]
    
    for i in output:
        print(f'{i} : {i:b}')
    print( f'합계: {sum(output)}')
if __name__ == '__main__':
    main()
