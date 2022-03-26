package globals.utils.json;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.URL;
import java.net.URLConnection;

import org.json.simple.*;
import org.json.simple.parser.*;

public class JsonReader {
  public static JSONObject getJsonDataFromReader(FileReader in) {
    JSONParser parser = new JSONParser();

    try {
      return (JSONObject) parser.parse(in);
    } catch (ParseException | IOException e) {
      e.printStackTrace();
    }
    return null;
  }

  public static JSONObject getJsonDataFromBufferReader(BufferedReader in) {
    JSONParser parser = new JSONParser();

    try {
      return (JSONObject) parser.parse(in);
    } catch (ParseException | IOException e) {
      e.printStackTrace();
    }
    return null;
  }

  public static JSONObject getJsonDataFromURL(String url) {
    try {
      URL netUrl = new URL(url); // URL to Parse
      URLConnection yc = netUrl.openConnection();
      return getJsonDataFromBufferReader(new BufferedReader(new InputStreamReader(yc.getInputStream())));
    } catch (IOException e) {
      e.printStackTrace();
    }
    return null;
  }

  public static JSONObject getJsonDataFromFile(String path) {
    try {
      return getJsonDataFromReader(new FileReader(path));
    } catch (FileNotFoundException e) {
      e.printStackTrace();
    }
    return null;
  }
}