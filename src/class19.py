def main():
    number = input('정수입력 :')
    last_character = number[-1]
    
    if last_character in '02468':
        print('짝수 입니다.')
    
    if last_character in '13579':
        print('홀수 입니다.')
    

if __name__ == '__main__':
    main()
