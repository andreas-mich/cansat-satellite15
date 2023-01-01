
InputNum = float(input('Number Input'))

count = 1

while True:
    count+=1
    if (InputNum == 0):
        print('Error')
        print('   ')
        break
        
    elif (InputNum == 1 or InputNum == 2):
        print(InputNum, 'is a prime number')
        print('   ')
        break
    else:
        x = InputNum/count
            
        if ((x.is_integer()) == True and x != 1):
            print(InputNum, 'is not a prime number')
            print('   ')
            break
        elif ((x.is_integer()) == True and x == 1):
            print(InputNum, 'is a prime number')
            print('   ')
            break
        


