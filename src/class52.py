def main():
    print('first line')
    while True:
        try:
            print('try')
            # raise
            break
            print('after break')
        except:
            print('except')
        finally:
            print('finally')
        print('end of while')
    print('end of main')
    
if __name__ == '__main__':
    main()
