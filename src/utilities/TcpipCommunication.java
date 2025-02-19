package utilities;

import java.io.DataOutputStream;
import java.io.IOException;
import java.io.PrintWriter;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.Scanner;

/**
 * This is the TCP/IP communication class. </br>
 * For the communication and data transfer, Robot is the server and Master PC (if communicating from PC) is client. </br>
 * Class constructor is two arguments - robot IP address and socket port number (IP address is not utilized for creating the server socket). </br>
 * For each class method description, description is added on the how client code should look like (in Python & MATLAB programming language). </br> 
 * To start connection, class object should call establishConnection() method first. </br>
 * Class object should always close the socket server with closeConenction() method. </br>
 */
public class TcpipCommunication {
	private int port;
	private String robot_ip_address;
	private ServerSocket serverSocket = null;
	private Socket socket = null;
	private int socket_timeout = 0;
	
	public TcpipCommunication(String robot_ip_address, int port, int socket_timeout)
	{
		this.robot_ip_address = robot_ip_address;
		this.port = port;
		this.socket_timeout = socket_timeout;
	}
	
	public TcpipCommunication(String robot_ip_address, int port )
	{
		this.robot_ip_address = robot_ip_address;
		this.port = port;
	}
	
	/**
	 * Establishes communication between robot server and client PC. </p>
	 * Client code (Python) - </p> 
	 * <code>
	 * import socket</br>
	 * import sys</br>
	 * import time</br>
	 * HOST = '<put server ip address>'</br>
	 * PORT = <put port number></br>
	 * sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)</br>
	 * a = sock.connect((HOST, PORT))</br>
	 * sock.sendall(str.encode("Establish Communication\r\n"))</br>
	 * data = sock.recv(1024)</br>
	 * if "Okay" in data:</br>
	 *	print("communication_established to the robot!")</br>
	 * </code>
	 * </p>
	 * Client code (MATLAB) - </p> 
	 * <code>
	 * HOST = '<put server ip address>';</br>
	 * PORT = <put port number>;</br>
	 * comm = tcpip(HOST,PORT,'NetworkRole','Client');</br>
	 * fopen(comm);</br>
	 * fprintf(comm,'Establish Communication');</br>
	 * data = fscanf(comm,'%s');</br>
	 * if (strcmp(data,'Okay')==1)</br>
	 * disp('communication_established to the robot!');</br>
	 * end</br>
	 * </code> 
	 */
	public void establishConnection()
	{
		try
		{
			serverSocket = new ServerSocket(port);
			serverSocket.setSoTimeout(socket_timeout);
			System.out.println("================================== \n" + 
					"Robot Server Socket opened at - \n\n" + 
					"host ip: " + robot_ip_address + "\n" +
					"port: " + serverSocket.getLocalPort() + 
					"\n==================================");
			 
			// waiting for client to connect to the robot server
			socket = serverSocket.accept();
			System.out.println("connection request received from the client...");
			Scanner connection_scanner = new Scanner(socket.getInputStream());
			String str =connection_scanner.nextLine();
			System.out.println(str);
			if (str.equals("Establish Communication"))
			{
				DataOutputStream connection_output = new DataOutputStream(socket.getOutputStream());
				PrintWriter connection_writer = new PrintWriter(connection_output, true);
				connection_writer.println("Okay");			
				System.out.println("Communication Established!");
			}
		}
		catch(IOException e)
		{
			System.out.println("Server exception while establishing connection: " + e.getMessage());
            e.printStackTrace();
		};
	}
	
	/**
	 * Receive data from the client. </p>
	 * Client code (Python)- </p>
	 * <code>
	 * sock.sendall(str.encode("Establish Communication\r\n"))</br>
	 * </code>
	 * </p>
	 * Client code (MATLAB)- </p>
	 * <code>
	 * fprintf(comm,data);</br>
	 * </code>
	 */
	public String getClientRequest()
	{
		String data = "";
		try
		{
			Scanner data_scan = new Scanner(socket.getInputStream());
			data = data_scan.nextLine();
		}
		catch(IOException e)
		{
			System.out.println("Server exception while receiving data : " + e.getMessage());
            e.printStackTrace();
		};
		return data;
	}
	
	/**
	 * Send data to the client.</p>
	 * Client code (Python)- </p>
	 * <code>
	 * data = sock.recv(1024)</br>
	 * </code>
	 * </p>
	 * Client code (MATLAB)- </p>
	 * <code>
	 * data = fscanf(comm,'%s');</br>
	 * </code>
	 */
	public void sendClientResponse(String resp_str)
	{
		try
		{
			DataOutputStream resp = new DataOutputStream(socket.getOutputStream());
            PrintWriter write_data = new PrintWriter(resp, true);
            write_data.println(resp_str);
		}
		catch(IOException e)
		{
			System.out.println("Server exception while sending data : " + e.getMessage());
            e.printStackTrace();
		};
		
	}
	
	/**
	 * Close the socket connection.</p>
	 * Client code (Python)- </p>
	 * <code>
	 * sock.close()</br>
	 * </code>
	 * </p>
	 * Client code (MATLAB)- </p>
	 * <code>
	 * delete(comm);</br>
	 * </code>
	 */
	public void closeConenction()
	{
		try
		{	
			socket.close();
            serverSocket.close();
		}
		catch(IOException e)
		{
			System.out.println("Server exception while closing socket : " + e.getMessage());
            e.printStackTrace();
		}
		finally 
		{
		    try 
		    {  
		    	if (!socket.isClosed()){
		    		socket.close();
		    	}
		    	if (!serverSocket.isClosed()){
		    		serverSocket.close();
		    	}
		    	
		    } 
		    catch(Exception e) {}
		}
	}
		
		
		
		
}

/**
 * OLD CLASS (robot was client and PC was server)
 * <pre>
 * public class TcpipCommunicationOld 
 * {
 * private String host_address;
 * private int port;
 * private Socket socket;
 * TcpipCommunicationOld(String Host_Address,int Port)
 * {
 * host_address = Host_Address;
 * port = Port;
 * }
 * 	
 * public void connectToSocket(String host, int port) 
 * {
 * String Host = host;
 * int Port = port;
 * try 
 * {
 * socket = new Socket(Host, Port);
 * }
 * catch (Exception e) 
 * { 
 * System.out.println("Failure to Connect to Python due to:  "+e); 
 * }
 * 
 * while (socket==null)
 * {
 * try 
 * {
 * socket = new Socket(Host, Port);
 * }
 * 
 * catch (Exception e) { System.out.println("Failure to Connect to Python due to:  "+e); }
 * }
 * System.out.println("Conenction Established to Python");
 * }
 * 
 * public void sendToSocket(String msg)
 * {
 * String message = msg;
 * msg = msg + "\r\n";
 * 
 * 
 * try 
 * {
 * PrintStream p = new PrintStream(socket.getOutputStream());
 * p.println(message);
 * }
 * catch (Exception e) { System.out.println("Failure to Send Packets to Python due to:  "+e); }
 * }
 *
 * public String receiveFromSocket()
 * {
 *String reply="";
 *
 *System.out.print("Waiting for Move Command");
 *try
 *{
 *Scanner sc1 = new Scanner(socket.getInputStream());
 *reply=sc1.nextLine();
 *}
 *catch (Exception e) { System.out.println("Failure to Receive Packets from Python due to:  "+e); }
 *return reply;
 *}
 *
 *public void closeSocket()
 *{
 *try 
 *{
 *socket.close();
 *} 
 *catch (Exception e) {  System.out.println("Failure to Close the Socket due to:  "+e);  }
 *}
 *
 *public void establishConnection() {
 *System.out.println("Establishing Connection with PC on Port:   " + port);
 *connectToSocket(host_address,port);
 *String reply = receiveFromSocket();
 *if (reply.equals("Prepare")) 
 *{
 *System.out.println("Conenction established......");
 *System.out.println("\n/////  Awaiting instructions  /////\n ");
 *}
 *else 
 *{ 
 *Timedelay.wait_minutes(1000);
 *}
 *sendToSocket("return");
 *}
* }
* <pre>
*/
