#include <iostream>
#include <string>

using namespace std;
string protocol = "http://ecn.t";
string domain   = ".tiles.virtualearth.net/tiles/";
string suffix   = ".jpeg?g=563&mkt=en-IN";

void download(string server, string tile) {
  string cmd = "while [ 1 ]; do wget -c \"" + protocol + server + domain + tile + suffix + "\" -O " + tile + ".jpeg; if [ $? -eq 0 ]; then break; fi; done";
  cout << cmd << endl;
}

void download_recursive(string server, string tile) {
  download(server, tile);
  if (tile.length() == 21) return;
  download_recursive("0", tile + "0");
  download_recursive("1", tile + "1");
  download_recursive("2", tile + "2");
  download_recursive("3", tile + "3");
}

int main() {
  string tile      = "a12330031112110";
  cout << "#!/bin/bash" << endl;
  download_recursive("0", tile);
}

