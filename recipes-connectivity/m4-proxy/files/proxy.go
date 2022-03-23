package main

import (
	"bytes"
	"fmt"
	"io"
	"net"
	"os"
	"reflect"
	"strconv"
	"time"

	"github.com/msgpack-rpc/msgpack-rpc-go/rpc"

	"github.com/facchinm/msgpack-go"
)

const (
	REQUEST      = 0
	RESPONSE     = 1
	NOTIFICATION = 2
)

type Resolver map[string]reflect.Value

func handleConnection(c net.Conn, chardev *os.File, resp chan []byte) {

	fmt.Printf("Serving %s\n", c.RemoteAddr().String())

	data, _, err := msgpack.Unpack(c)
	if err != nil {
		fmt.Println(err)
		return
	}

	fmt.Println(data)
	var buf bytes.Buffer

	msgpack.Pack(&buf, data.Interface())

	/*
		msgId := _req[1]
		msgName := _req[2]
		msgArgs := _req[3]

		rawdata := make([]byte, 5)
		rawdata[0] = byte(msgType.Int())
		rawdata[1] = byte(msgId.Int())
		rawdata[2] = byte(msgId.Int())
		rawdata[3] = byte(msgId.Int())
		rawdata[4] = byte(msgId.Int())
		rawdata = append(rawdata, msgName.Bytes()...)

		something := msgArgs.Addr().Bytes()

		fmt.Println(something)
		rawdata = append(rawdata, something...)

		fmt.Println(data)
		fmt.Println(rawdata)
	*/

	fmt.Println(buf)

	chardev.Write(buf.Bytes())

	msgType := buf.Bytes()[1]

	if msgType == REQUEST {
		// wait to be unlocked by the other reading goroutine
		// TODO: add timeout handling
		fmt.Println("wait for response")
		select {
		case response := <-resp:
			//chardev.Read(response)
			fmt.Println("return response to client")
			c.Write(response)
		case <-time.After(1 * time.Second):
			c.Write(nil)
		}
	}
	fmt.Println("done")

	if msgType == NOTIFICATION {
		// fire and forget
	}

	c.Close()
}

func chardevListener(chardev *os.File, resp chan []byte) {

	for {

		fmt.Println("charDevListener")

		data := make([]byte, 1024)
		response := make([]byte, 1024)

		n, err := chardev.Read(data)

		data = data[:n]

		if err != nil {
			fmt.Println(err)
			continue
		}
		fmt.Println("chardev.Read returned")

		if n <= 0 {
			continue
		}

		fmt.Println("got data from chardev")
		fmt.Println(data)

		start := 0
		for {

			fmt.Println("unpacker loop")

			copy_data := data[start:]

			message, n, err := msgpack.UnpackReflected(bytes.NewReader(copy_data))

			start += n

			fmt.Printf("%d bytes consumed\n", n)
			fmt.Printf("%v\n", message)
			fmt.Println(message)
			fmt.Println(err)

			if err == io.EOF {
				break
			}

			_req, ok := message.Interface().([]reflect.Value)
			if !ok {
				continue
			}

			msgType := _req[0]

			fmt.Println("before response")

			if msgType.Int() == RESPONSE {
				fmt.Println("got response and continue")
				// unlock thread waiting on handleConnection
				resp <- copy_data[:n]
				continue
			}

			fmt.Println("before serving")

			msgFunction := _req[2]
			port := functionToPort(string(msgFunction.Bytes()))

			fmt.Println("Serving function ", string(msgFunction.Bytes()), " to port ", port)

			// REQUEST or NOTIFICATION
			conn, err := net.Dial("tcp", port)
			if err != nil {
				fmt.Println(err)
				continue
			}
			conn.Write(copy_data[:n])

			if msgType.Int() == REQUEST {
				fmt.Println("ask for a response")

				var to_send []byte
				i := 0
				for {
					n, err := conn.Read(response)
					conn.SetReadDeadline(time.Now().Add(100 * time.Millisecond))
					to_send = append(to_send, response[:n]...)
					i += n
					if err != nil {
						break
					}
				}
				fmt.Println("sending ", to_send[:i])
				chardev.Write(to_send[:i])
			}

			if msgType.Int() == NOTIFICATION {
				// fire and forget
			}

			conn.Close()
		}
	}
}

func (self Resolver) Resolve(name string, arguments []reflect.Value) (reflect.Value, error) {
	fmt.Println("resolving ", name)
	return self[name], nil
}

func (self Resolver) Functions() []string {
	var functions []string
	for el := range self {
		functions = append(functions, el)
	}
	fmt.Println(functions)
	return functions
}

var functions map[string]int

func functionToPort(function string) string {
	return ":" + strconv.Itoa(functions[function])
}

func register(port uint, arg []reflect.Value) string {
	for _, elem := range arg {
		functions[string(elem.Bytes())] = int(port)
	}
	fmt.Println("Registering service on port ", port, " with functions ", functions)
	return ""
}

func main() {

	functions = make(map[string]int)

	chardev, err := os.OpenFile("/dev/x8h7_ui", os.O_RDWR, 0)

	chardev_reader_chan := make(chan []byte, 1024)

	l, err := net.Listen("tcp4", ":5001")
	if err != nil {
		fmt.Println(err)
		return
	}
	defer l.Close()

	res := Resolver{"register": reflect.ValueOf(register)}

	go chardevListener(chardev, chardev_reader_chan)

	serv := rpc.NewServer(res, true, nil, 5000)
	lx, _ := net.Listen("tcp", ":5000")
	serv.Listen(lx)
	go serv.Run()

	for {
		c, err := l.Accept()
		if err != nil {
			fmt.Println(err)
			return
		}
		go handleConnection(c, chardev, chardev_reader_chan)
	}
}
