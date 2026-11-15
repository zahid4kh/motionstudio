// MotionStudio robot API for Kotlin / the JVM. No runtime dependencies.
//   ./gradlew build          library jar
//   ./gradlew test           live tests (start build/motionstudio-server of the repository)
//   ./gradlew runExample -Pexample=Square
plugins {
    kotlin("jvm") version "2.4.21"
    `java-library`
    `maven-publish`
}

group = "io.motionstudio"
version = "0.1.0"

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
    testLogging { events("passed", "skipped", "failed"); exceptionFormat = org.gradle.api.tasks.testing.logging.TestExceptionFormat.FULL }
}

tasks.register<JavaExec>("runExample") {
    group = "application"
    description = "Runs an example: -Pexample=Square (or HumanoidWave, TwoMotions)"
    classpath = sourceSets["examples"].runtimeClasspath
    mainClass.set("examples.${project.findProperty("example") ?: "Square"}Kt")
}

java { withSourcesJar() }

publishing {
    publications { create<MavenPublication>("maven") { from(components["java"]) } }
}
