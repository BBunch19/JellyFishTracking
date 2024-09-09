import socket
import os

def send_images(directory, host, port):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    print('connecting')
    s.connect((host, port))
    print('connected')
    
    for filename in os.listdir(directory):
        filepath = os.path.join(directory,filename)
        if os.path.isfile(filepath) and filename.lower().endswith(('.png', '.jpg','.jpeg','.gif','.bmp')):
            with open(filepath, 'rb') as f:
                image_data = f.read()
                s.sendall(filename.encode() + b'\n')
                s.sendall(image_data)
                s.sendall(b'\nEOF\n')
            os.remove(filepath)
            print(f'Sent and deleted: {filename}')

    s.close()



my_ip = '10.29.156.85'
port = 12345
send_images('savedcopy',my_ip,port)