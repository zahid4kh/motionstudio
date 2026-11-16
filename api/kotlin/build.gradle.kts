// MotionStudio robot API for Kotlin / the JVM. No runtime dependencies.
//   ./gradlew build          library jar
//   ./gradlew test           live tests (start build/motionstudio-server of the repository)
//   ./gradlew runExample -Pexample=Square
//   ./gradlew centralBundle  signed bundle for Maven Central (build/distributions/…-bundle.zip)
plugins {
    kotlin("jvm") version "2.4.21"
    `java-library`
    `maven-publish`
    signing
}

group = "io.github.zahid4kh"
version = "0.2.0"

repositories { mavenCentral() }

kotlin { jvmToolchain(17) }

sourceSets {
    create("examples") {
        compileClasspath += sourceSets.main.get().output
        runtimeClasspath += sourceSets.main.get().output
    }
}

dependencies {
    testImplementation(kotlin("test"))
    "examplesImplementation"(kotlin("stdlib"))
}

tasks.test {
    useJUnitPlatform()
    systemProperty("motionstudio.server",
        System.getenv("MOTIONSTUDIO_SERVER") ?: rootDir.resolve("../../build/motionstudio-server").canonicalPath)
    systemProperty("motionstudio.template",
        rootDir.resolve("../../templates/indexing_table.mstudio").canonicalPath)
    testLogging { events("passed", "skipped", "failed"); exceptionFormat = org.gradle.api.tasks.testing.logging.TestExceptionFormat.FULL }
}

tasks.register<JavaExec>("runExample") {
    group = "application"
    description = "Runs an example: -Pexample=Square (or HumanoidWave, TwoMotions, WalkSquare, WalkPatrol)"
    classpath = sourceSets["examples"].runtimeClasspath
    mainClass.set("examples.${project.findProperty("example") ?: "Square"}Kt")
}

java {
    withSourcesJar()
    withJavadocJar()
}

publishing {
    publications {
        create<MavenPublication>("maven") {
            from(components["java"])
            pom {
                name.set("MotionStudio robot API")
                description.set("Control the robots, humanoids and stations of a MotionStudio scene from Kotlin or Java: straight-line and joint moves, taught poses, walking, signals.")
                url.set("https://github.com/zahid4kh/motionstudio")
                licenses {
                    license {
                        name.set("MIT License")
                        url.set("https://opensource.org/licenses/MIT")
                    }
                }
                developers {
                    developer {
                        id.set("zahid4kh")
                        name.set("Zahid Khalilov")
                        url.set("https://github.com/zahid4kh")
                    }
                }
                scm {
                    url.set("https://github.com/zahid4kh/motionstudio")
                    connection.set("scm:git:https://github.com/zahid4kh/motionstudio.git")
                    developerConnection.set("scm:git:ssh://git@github.com/zahid4kh/motionstudio.git")
                }
            }
        }
    }
    // Maven Central takes a zip of a Maven repository layout: built here, uploaded by hand
    // at central.sonatype.com › Publish Component.
    repositories { maven { name = "staging"; url = uri(layout.buildDirectory.dir("staging")) } }
}

// Signs with the gpg command and its agent: pass -Psigning.gnupg.keyName=<key id>
// (or put it in ~/.gradle/gradle.properties).
signing {
    useGpgCmd()
    sign(publishing.publications["maven"])
}
tasks.withType<Sign>().configureEach {
    onlyIf { gradle.taskGraph.hasTask(":centralBundle") }
    doFirst {
        if (project.findProperty("signing.gnupg.keyName") == null)
            throw GradleException("Set the signing key: -Psigning.gnupg.keyName=<gpg key id>")
    }
}

tasks.register<Zip>("centralBundle") {
    group = "publishing"
    description = "Builds the signed bundle to upload to the Maven Central Portal"
    val staging = layout.buildDirectory.dir("staging")
    dependsOn("publishMavenPublicationToStagingRepository")
    from(staging) {
        include("${project.group.toString().replace('.', '/')}/${project.name}/${project.version}/**")
    }
    archiveFileName.set("${project.name}-${project.version}-bundle.zip")
    destinationDirectory.set(layout.buildDirectory.dir("distributions"))
}
tasks.named("publishMavenPublicationToStagingRepository") {
    doFirst { delete(layout.buildDirectory.dir("staging")) }
}
