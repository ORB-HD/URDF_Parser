pipeline {
  agent any
  stages {
    stage('Build') {
      steps {
        cmakeBuild(installation: 'InSearchPath', buildDir: 'build', buildType: 'Debug', cleanBuild: true, cmakeArgs: '-DURDF_BUILD_TEST=On')
        dir(path: 'build') {
          sh 'make -j 8'
        }

      }
    }

    stage('Test Library') {
      steps {
        dir(path: 'build') {
          sh './test_library -r junit > testresults.xml'
        }

      }
    }
  }
  
  post {
    always {
      junit 'build/testresults.xml'
    }
  }
}
