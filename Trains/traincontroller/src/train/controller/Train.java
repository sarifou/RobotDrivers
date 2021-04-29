/**
 * @author Mamadou Sarifou DIALLO
 *
 */
package train.controller;

// Import des bibliothèques
import java.net.Socket;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.BufferedReader;




public class Train {


	private String ipAddress ;
	
	private int port ;
	
	private int trainAddress ;
	
	private Socket  socket_client;

	private BufferedReader input;
	
	private DataOutputStream output ;
	
	private String response ;
	
	
	public Train(String ip, int port, int trainNumber) {
		this.ipAddress = ip ;
		this.port = port ;
		this.trainAddress = trainNumber ;
	}
	
	public void publish(String request) {
		
		byte[] requestByte = request.getBytes();
		
		try {
			
			socket_client = new Socket(this.ipAddress, this.port);
			
			output = new DataOutputStream(socket_client.getOutputStream());
			
			output.write(requestByte);
			
			output.flush();
			
			input = new BufferedReader(new InputStreamReader(socket_client.getInputStream()));

			this.response = input.readLine();
			
			System.out.println(this.response);
			
			output.close();
			input.close();
			socket_client.close();
		} catch (IOException e) {
			System.out.println("Erreur d'envoi des données");
		}
	}
	
	public void command(String command) {
		String request = "{ \"train\" :" 
				+ this.trainAddress + 
				", \"command\" : \"" + 
				command + "\"}";
		this.publish(request);
	}
	public void command_bool(String command, boolean state) {
		String request = "{ \"train\" :" 
				+ this.trainAddress + 
				", \"command\" : \"" + 
				command + "\", \"state\" :"
				+ state +"}";
		this.publish(request);
	}
	public void faster() throws IOException {
		this.command("faster");
	}
	public void slower() throws IOException {
		this.command("slower");
		
	}
	public void reverse() throws IOException {
		this.command("reverse");
		
	}
	public void stop() throws IOException {
		this.command("stop");	
	}
	public void setSpeed(int speed) throws IOException {
		String request = "{ \"train\" :" 
				+ this.trainAddress + 
				", \"command\" : \"" + 
				"speed" + "\", \"value\" :"
				+ speed +"}";
		this.publish(request);
	}
	
	public void light(boolean state)  {
		this.command_bool("fl", state);
	}
	public void train_noise(boolean state) {
		this.command_bool("f1", state);
	}
	public void wagon_noise(boolean state) {
		this.command_bool("f2", state);
	}
	public void klaxon(boolean state) {
		this.command_bool("f3", state);
	}
	public String getIp() {
		return ipAddress;
	}

	public void setIp(String ipAddress) {
		this.ipAddress = ipAddress;
	}

	public int getPort() {
		return port;
	}

	public void setPort(int port) {
		this.port = port;
	}

	public int getTrainNumber() {
		return trainAddress;
	}

	public void setTrainNumber(int trainAddress) {
		this.trainAddress = trainAddress;
	}
	
	public String getResponse()  {
		return response ;
	}
	
}
