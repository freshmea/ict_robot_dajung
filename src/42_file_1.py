def main():
    # f = open('data/text.txt', 'w')
    # f.write('Hello Python Programming...!')
    # f.close()
    
    with open('data/text.txt', 'a') as f:
        f.write('Hello Python Programming... this is with open!\n')

if __name__ == '__main__':
    main()
