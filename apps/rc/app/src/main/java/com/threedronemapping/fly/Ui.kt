package com.threedronemapping.fly

import android.content.Context
import android.graphics.Typeface
import android.graphics.drawable.GradientDrawable
import android.text.InputType
import android.util.TypedValue
import android.view.Gravity
import android.view.ViewGroup
import android.widget.Button
import android.widget.EditText
import android.widget.LinearLayout
import android.widget.TextView

/** Tiny programmatic-view toolkit: dark, high-contrast, big touch targets for a 7" RC in sunlight. */
object Ui {
    const val BG = 0xFF0B0D10.toInt()
    const val CARD = 0xFF161A20.toInt()
    const val TEXT = 0xFFF2F4F7.toInt()
    const val DIM = 0xFF9AA4B2.toInt()
    const val ACCENT = 0xFFFFB000.toInt() // amber: readable in sun
    const val GOOD = 0xFF3FD37A.toInt()
    const val BAD = 0xFFFF5A52.toInt()
    const val BTN = 0xFF2A3340.toInt()
    const val BTN_DANGER = 0xFF6B1F1B.toInt()

    val MONO: Typeface = Typeface.MONOSPACE

    fun dp(ctx: Context, v: Int): Int =
        TypedValue.applyDimension(TypedValue.COMPLEX_UNIT_DIP, v.toFloat(), ctx.resources.displayMetrics).toInt()

    fun vertical(ctx: Context): LinearLayout = LinearLayout(ctx).apply { orientation = LinearLayout.VERTICAL }

    fun row(ctx: Context): LinearLayout = LinearLayout(ctx).apply {
        orientation = LinearLayout.HORIZONTAL
        gravity = Gravity.CENTER_VERTICAL
    }

    /** A titled card; returns the card body to add content to. */
    fun section(parent: LinearLayout, number: String, title: String, note: String? = null): LinearLayout {
        val ctx = parent.context
        val card = vertical(ctx).apply {
            background = GradientDrawable().apply { setColor(CARD); cornerRadius = dp(ctx, 10).toFloat() }
            setPadding(dp(ctx, 14), dp(ctx, 12), dp(ctx, 14), dp(ctx, 12))
        }
        card.addView(TextView(ctx).apply {
            text = "$number  $title"
            setTextColor(ACCENT)
            setTextSize(TypedValue.COMPLEX_UNIT_SP, 20f)
            typeface = Typeface.DEFAULT_BOLD
        })
        if (note != null) card.addView(TextView(ctx).apply {
            text = note
            setTextColor(DIM)
            setTextSize(TypedValue.COMPLEX_UNIT_SP, 14f)
            setPadding(0, dp(ctx, 2), 0, dp(ctx, 6))
        })
        parent.addView(card, LinearLayout.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.WRAP_CONTENT).apply {
            bottomMargin = dp(ctx, 12)
        })
        return card
    }

    /** "label: value" line; returns the value TextView. */
    fun kv(parent: LinearLayout, label: String, initial: String = "—"): TextView {
        val ctx = parent.context
        val r = row(ctx).apply { setPadding(0, dp(ctx, 3), 0, dp(ctx, 3)) }
        r.addView(TextView(ctx).apply {
            text = label
            setTextColor(DIM)
            setTextSize(TypedValue.COMPLEX_UNIT_SP, 16f)
        }, LinearLayout.LayoutParams(dp(ctx, 190), ViewGroup.LayoutParams.WRAP_CONTENT))
        val v = TextView(ctx).apply {
            text = initial
            setTextColor(TEXT)
            setTextSize(TypedValue.COMPLEX_UNIT_SP, 17f)
            typeface = MONO
        }
        r.addView(v, LinearLayout.LayoutParams(0, ViewGroup.LayoutParams.WRAP_CONTENT, 1f))
        parent.addView(r)
        return v
    }

    fun text(parent: LinearLayout, initial: String = "", sizeSp: Float = 15f, color: Int = TEXT, mono: Boolean = true): TextView {
        val v = TextView(parent.context).apply {
            text = initial
            setTextColor(color)
            setTextSize(TypedValue.COMPLEX_UNIT_SP, sizeSp)
            if (mono) typeface = MONO
        }
        parent.addView(v)
        return v
    }

    /** A row of equally wide, tall buttons. */
    fun buttons(parent: LinearLayout, vararg specs: Pair<String, () -> Unit>): List<Button> {
        val ctx = parent.context
        val r = row(ctx).apply { setPadding(0, dp(ctx, 6), 0, dp(ctx, 2)) }
        val out = specs.map { (label, action) ->
            val danger = label.startsWith("Stop", true) || label.startsWith("Disable", true)
            Button(ctx).apply {
                text = label
                isAllCaps = false
                setTextColor(TEXT)
                setTextSize(TypedValue.COMPLEX_UNIT_SP, 17f)
                minHeight = dp(ctx, 60)
                background = GradientDrawable().apply {
                    setColor(if (danger) BTN_DANGER else BTN)
                    cornerRadius = dp(ctx, 8).toFloat()
                }
                setOnClickListener {
                    try { action() } catch (t: Throwable) { Phase0Log.e("UI", "button '$label' failed", t) }
                }
            }.also { b ->
                r.addView(b, LinearLayout.LayoutParams(0, dp(ctx, 60), 1f).apply {
                    marginStart = dp(ctx, 4); marginEnd = dp(ctx, 4)
                })
            }
        }
        parent.addView(r)
        return out
    }

    fun input(parent: LinearLayout, label: String, initial: String): EditText {
        val ctx = parent.context
        val r = row(ctx)
        r.addView(TextView(ctx).apply {
            text = label
            setTextColor(DIM)
            setTextSize(TypedValue.COMPLEX_UNIT_SP, 16f)
        }, LinearLayout.LayoutParams(dp(ctx, 190), ViewGroup.LayoutParams.WRAP_CONTENT))
        val e = EditText(ctx).apply {
            setText(initial)
            setTextColor(TEXT)
            setTextSize(TypedValue.COMPLEX_UNIT_SP, 18f)
            typeface = MONO
            inputType = InputType.TYPE_CLASS_NUMBER or InputType.TYPE_NUMBER_FLAG_DECIMAL or InputType.TYPE_NUMBER_FLAG_SIGNED
            minHeight = dp(ctx, 56)
            setSingleLine()
        }
        r.addView(e, LinearLayout.LayoutParams(0, ViewGroup.LayoutParams.WRAP_CONTENT, 1f))
        parent.addView(r)
        return e
    }

    fun colorFor(ok: Boolean?): Int = when (ok) { true -> GOOD; false -> BAD; null -> TEXT }
}
