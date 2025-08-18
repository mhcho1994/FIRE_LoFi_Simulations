# What is this

This is the most basic example of the api that cromuelence uses inside our replicant models to talk with modelica models and fmu's.

To keep things simple we did **not** include the replicant wrapper but instead just the stuff we use to interact with modelica/fmu models. 

We suggest running the example with docker 
# How to run using docker  

First build the docker container 

```
docker build -t example . 
```

Run it with docker 

```
docker run -it example
```


# How to run without docker 

These instructions should work pretty well but could be dependent on your install of miniconda.

Using miniconda 

Apt install open modleica

```
apt update 
apt install -y ca-certificates curl gnupg git
curl -fsSL http://build.openmodelica.org/apt/openmodelica.asc |  gpg --dearmor -o /usr/share/keyrings/openmodelica-keyring.gpg 
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/openmodelica-keyring.gpg] https://build.openmodelica.org/apt $(cat /etc/os-release | grep "\(UBUNTU\\|DEBIAN\\|VERSION\)_CODENAME" | sort | cut -d= -f 2 | head -1) stable" |  tee /etc/apt/sources.list.d/openmodelica.list 
apt update 
apt install -y openmodelica
```

Install the following conda package:
```
conda install -c conda-forge pyfmi 
```

Install the following pip packages 

```
pip3 install pandas OMPython 
```

Run it 

```
python3 eample.py
```