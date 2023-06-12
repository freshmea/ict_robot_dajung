def main():
    print(range(10)) # 제너레이터
    for i in range(10):
        print('출력')
    # for(int i = 0 ; i < 10; i++){} c code
    
    
    list_a = ['choi', 'su', 'gil', 1234, 123.2312]
    for i in list_a:
        print(i)
    
    for i, value in enumerate(list_a):
        print(i, value)
    

if __name__ == '__main__':
    main()
