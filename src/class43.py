def main():
    # with open('data/text.txt', 'r') as f:
    #     data = f.read()
    # print(data)
        
    # with open('data/text.txt', 'r') as f:
    #     data = ' '
    #     while(data):
    #         data = f.readline()
    #         print(data, end='')
    
    with open('data/text.txt', 'r') as f:
        data = f.readlines()
        print(data)
        for str in data:
            print(str, end='')

if __name__ == '__main__':
    main()
