package io.motionstudio

// A small JSON reader and writer for the API's messages: objects are Maps,
// arrays Lists, numbers Doubles. Internal to the library.
// A null that is sent (params with plain null values are left out)
internal object JsonNull

internal object Json {
    fun write(v: Any?): String = StringBuilder().also { write(it, v) }.toString()

    private fun write(out: StringBuilder, v: Any?) {
        when (v) {
            null, JsonNull -> out.append("null")
            is Boolean -> out.append(v)
            is Number -> {
                val d = v.toDouble()
                if (d.isNaN() || d.isInfinite()) out.append("null")
                else if (d == Math.rint(d) && Math.abs(d) < 1e15) out.append(d.toLong()) else out.append(d)
            }
            is String -> writeString(out, v)
            is Map<*, *> -> {
                out.append('{')
                var first = true
                for ((k, value) in v) {
                    if (!first) out.append(',')
                    first = false
                    writeString(out, k.toString())
                    out.append(':')
                    write(out, value)
                }
                out.append('}')
            }
            is Iterable<*> -> {
                out.append('[')
                var first = true
                for (value in v) {
                    if (!first) out.append(',')
                    first = false
                    write(out, value)
                }
                out.append(']')
            }
            else -> writeString(out, v.toString())
        }
    }

    private fun writeString(out: StringBuilder, s: String) {
        out.append('"')
        for (c in s) {
            when (c) {
                '"' -> out.append("\\\"")
                '\\' -> out.append("\\\\")
                '\n' -> out.append("\\n")
                '\r' -> out.append("\\r")
                '\t' -> out.append("\\t")
                else -> if (c < ' ') out.append(String.format("\\u%04x", c.code)) else out.append(c)
            }
        }
        out.append('"')
    }

    fun parse(text: String): Any? {
        val p = Parser(text)
        val v = p.value()
        p.skip()
        if (p.i != text.length) throw IllegalArgumentException("JSON: trailing characters")
        return v
    }

    private class Parser(val s: String) {
        var i = 0

        fun skip() { while (i < s.length && s[i].isWhitespace()) i++ }

        fun value(): Any? {
            skip()
            if (i >= s.length) throw IllegalArgumentException("JSON: unexpected end")
            return when (s[i]) {
                '{' -> obj()
                '[' -> arr()
                '"' -> str()
                't' -> literal("true", true)
                'f' -> literal("false", false)
                'n' -> literal("null", null)
                else -> num()
            }
        }

        private fun literal(word: String, v: Any?): Any? {
            if (!s.startsWith(word, i)) throw IllegalArgumentException("JSON: unexpected character at $i")
            i += word.length
            return v
        }

        private fun num(): Double {
            val start = i
            while (i < s.length && (s[i].isDigit() || s[i] in "+-.eE")) i++
            return s.substring(start, i).toDoubleOrNull() ?: throw IllegalArgumentException("JSON: bad number at $start")
        }

        private fun str(): String {
            i++
            val out = StringBuilder()
            while (i < s.length && s[i] != '"') {
                val c = s[i++]
                if (c != '\\') { out.append(c); continue }
                when (val e = s[i++]) {
                    'n' -> out.append('\n')
                    't' -> out.append('\t')
                    'r' -> out.append('\r')
                    'b' -> out.append('\b')
                    'f' -> out.append('\u000c')
                    'u' -> { out.append(s.substring(i, i + 4).toInt(16).toChar()); i += 4 }
                    else -> out.append(e)
                }
            }
            if (i >= s.length) throw IllegalArgumentException("JSON: unterminated string")
            i++
            return out.toString()
        }

        private fun obj(): Map<String, Any?> {
            i++
            val out = LinkedHashMap<String, Any?>()
            skip()
            if (s[i] == '}') { i++; return out }
            while (true) {
                skip()
                val key = str()
                skip()
                if (s[i++] != ':') throw IllegalArgumentException("JSON: expected ':'")
                out[key] = value()
                skip()
                when (s[i++]) {
                    ',' -> continue
                    '}' -> return out
                    else -> throw IllegalArgumentException("JSON: expected ',' or '}'")
                }
            }
        }

        private fun arr(): List<Any?> {
            i++
            val out = ArrayList<Any?>()
            skip()
            if (s[i] == ']') { i++; return out }
            while (true) {
                out.add(value())
                skip()
                when (s[i++]) {
                    ',' -> continue
                    ']' -> return out
                    else -> throw IllegalArgumentException("JSON: expected ',' or ']'")
                }
            }
        }
    }
}
