pipeline {

    agent none

    stages {

        //////////////////////////////////////////////////
        // BUILD
        //////////////////////////////////////////////////
        stage('build'){
            agent {
                docker {
                    image 'gcc:8.4'
                    args '--user root'
                }
            }
            when{
                anyOf { branch 'master'; branch 'debug' }
                //changeset "**/*.c"
            }
            steps{
                echo 'Building radio-weather'
                sh 'apt-get update'
                sh 'apt-get -y install cmake'
                sh 'cmake -B ci-build'
                dir('ci-build'){
                    sh 'make'
                }
                archiveArtifacts artifacts: '**/ci-build/radio-weather.uf2', fingerprint: true
            }
        }

        //////////////////////////////////////////////////
        // SONAR TESTING
        //////////////////////////////////////////////////
        stage('Sonarqube') {
            when {
                //anyOf { branch 'master'; branch 'development' }
                changeset "**/src/**"
            }
            agent any
            environment{
              sonarpath = tool 'SonarScanner'
            }
            steps {
                echo 'Preparing for Sonarqube, compile with wrapper'
                sh 'cmake -B sonar-build'
                sh 'mkdir sonar-build/sonar'
                dir('sonar-build/sonar'){
                    sh 'wget "https://sonar.mj0.uk/static/cpp/build-wrapper-linux-x86.zip"'
                    sh 'unzip build-wrapper-linux-x86.zip'
                }
                dir('sonar-build'){
                    sh './sonar/build-wrapper-linux-x86/build-wrapper-linux-x86-64 --out-dir build_wrapper_output_directory make'
                }
                echo 'Running Sonarqube Analysis..'
                withSonarQubeEnv('SonarQube Developer') {
                    sh "${sonarpath}/bin/sonar-scanner -Dproject.settings=sonar-project.properties -Dsonar.branch.name=$BRANCH_NAME"
                }
                cleanWs()
            }
        }

        //////////////////////////////////////////////////
        // END STEPS
        //////////////////////////////////////////////////

    }

    post {
        always {
            echo 'Pipeline for result is complete...'
        }
        failure {
            echo 'Pipeline for result failed...'
            //slacksend (bla)
        }
        success {
            echo 'Pipeline for result succeeded...'
            //slacksend (bla)
        }
    }

}

