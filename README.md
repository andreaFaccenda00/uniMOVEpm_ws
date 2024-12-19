# uniMOVepm_ws

### 1. Configura rosdep

Prima di poter utilizzare `rosdep`, devi inizializzarlo ed eventualmente aggiornare il database:

```bash
sudo rosdep init
rosdep update
```

### 2. Clona il repository del progetto

Se il progetto non è già sul tuo sistema, clona il repository dalla sua origine (ad esempio, GitHub):

```bash
git clone <URL-del-repository>
cd <nome-cartella-repository>
```

### 3. Assicurati di trovarti nella root del workspace

Il comando `rosdep` deve essere eseguito dalla directory principale del tuo workspace. Se stai utilizzando il layout classico di ROS 2, assicurati di essere nella cartella del workspace (ad esempio `~/ros2_ws`):

```bash
cd ~/ros2_ws
```

### 4. Installa le dipendenze con rosdep

Esegui il comando seguente per installare tutte le dipendenze dichiarate nei file `package.xml` del workspace:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

- **Opzioni principali:**
  - `--from-paths src`: Specifica la cartella `src` come origine dei pacchetti.
  - `--ignore-src`: Ignora i pacchetti già presenti nel workspace.
  - `-r`: Risolvi le dipendenze ricorsivamente.
  - `-y`: Conferma automaticamente l'installazione.

### 5. Compila il workspace (facoltativo)

Dopo aver installato le dipendenze, puoi compilare il workspace:

```bash
colcon build
```

## Build from source

### Download all repositories

Per scaricare e configurare un progetto direttamente dal codice sorgente, segui questi passaggi:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/ros-controls/ros2_control_demos
cd ~/ros2_ws/
vcs import src < src/ros2_control_demos/ros2_control_demos.$ROS_DISTRO.repos
rosdep update --rosdistro=$ROS_DISTRO
sudo apt-get update
```

### Install dependencies

Esegui il seguente comando per installare le dipendenze:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

### Build everything

Infine, compila tutto utilizzando il comando:

```bash
. /opt/ros/${ROS_DISTRO}/setup.sh
colcon build --symlink-install
```





